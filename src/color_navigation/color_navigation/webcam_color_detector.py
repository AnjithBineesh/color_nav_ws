#!/usr/bin/env python3
"""
Laptop Webcam Color Detection Node for Autonomous Navigation.
Captures live video from the laptop webcam via OpenCV, classifies an object held
inside the central target box, and commands the simulated robot via /detected_color:
  - RED   -> Robot navigates to Red Pillar   at (2.5, 1.8)
  - GREEN -> Robot navigates to Green Pillar at (3.5, 0.0)
  - OTHER -> Robot navigates to White Pillar at (2.5, -1.8)

Also supports interactive keyboard shortcuts inside the webcam preview window:
  - [H] Send robot back to exact Home position (0.00, 0.00, Yaw=0.0 deg)
  - [R] Reset color lock / detector to standby
  - [Q] / [ESC] Quit node cleanly
"""

import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Header, String

from color_navigation.vision_utils import (
    bgr_to_ros_image,
    compute_color_masks,
    find_largest_contour_stats,
)

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


PILLAR_DESTINATION_LABEL = {
    'RED': 'RED PILLAR (2.5, 1.8)',
    'GREEN': 'GREEN PILLAR (3.5, 0.0)',
    'OTHER': 'WHITE PILLAR (2.5, -1.8)',
}


class WebcamColorDetector(Node):
    def __init__(self):
        super().__init__('webcam_color_detector')

        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        # Parameters
        self.declare_parameter('device_id', 0)
        self.declare_parameter('mirror', True)
        self.declare_parameter('show_window', True)
        self.declare_parameter('min_fill_ratio', 0.15)
        self.declare_parameter('required_confirmations', 10)
        self.declare_parameter('debug_topic', '/webcam/debug_image')
        self.declare_parameter('red_sat_min', 130)
        self.declare_parameter('green_sat_min', 85)
        self.declare_parameter('other_sat_min', 115)

        self.device_id = int(self.get_parameter('device_id').value)
        self.mirror = bool(self.get_parameter('mirror').value)
        self.show_window = bool(self.get_parameter('show_window').value)
        self.min_fill_ratio = float(self.get_parameter('min_fill_ratio').value)
        self.required_confirmations = int(self.get_parameter('required_confirmations').value)
        self.debug_topic = str(self.get_parameter('debug_topic').value)
        self.red_sat_min = int(self.get_parameter('red_sat_min').value)
        self.green_sat_min = int(self.get_parameter('green_sat_min').value)
        self.other_sat_min = int(self.get_parameter('other_sat_min').value)

        # Publishers & Subscribers
        self.color_pub = self.create_publisher(String, '/detected_color', 10)
        self.debug_pub = self.create_publisher(Image, self.debug_topic, 10)
        self.home_pub = self.create_publisher(Empty, '/return_home', 10)
        self.reset_pub = self.create_publisher(Empty, '/reset_detector', 10)
        self.active_pub = self.create_publisher(Empty, '/webcam_active', 10)

        self.reset_sub = self.create_subscription(Empty, '/reset_detector', self.reset_callback, 10)
        self.home_sub = self.create_subscription(Empty, '/return_home', self.reset_callback, 10)

        # Debouncing & stabilization state
        self.detection_history = []
        self.locked_decision = None
        self.empty_frames = 0
        self.standby_until_time = 0.0
        self.window_name = 'Laptop Webcam Color Detector'
        self.should_quit = False
        self.read_failures = 0

        # Open laptop webcam (auto-scanning fallback indices if device_id unavailable)
        self.cap, opened_id = self._open_webcam(self.device_id)
        if self.cap is None:
            self.get_logger().error(
                f'Could not open any laptop webcam (tried device_id={self.device_id} and indices 0..3). '
                'Check webcam permissions (/dev/video*) or close another app using the camera.'
            )
            self.should_quit = True
            return

        self.get_logger().info('=====================================================')
        self.get_logger().info(f'📷 Laptop Webcam Detector active on /dev/video{opened_id}')
        self.get_logger().info('   Hold colored object inside the central cyan Target Box:')
        self.get_logger().info('   - RED   -> Robot navigates to RED PILLAR')
        self.get_logger().info('   - GREEN -> Robot navigates to GREEN PILLAR')
        self.get_logger().info('   - OTHER -> Robot navigates to WHITE PILLAR')
        self.get_logger().info(f'   Debug image topic: {self.debug_topic}')
        if self.show_window:
            self.get_logger().info("   Window controls: [H] Home | [R] Reset | [Q] Quit")
        self.get_logger().info('=====================================================')

        # 30 Hz capture/processing timer
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)

    def _open_webcam(self, preferred_id: int):
        """Try opening preferred camera index, falling back cleanly across indices 0..3."""
        candidate_ids = [preferred_id] + [i for i in (0, 1, 2, 3) if i != preferred_id]
        for cam_id in candidate_ids:
            for backend in (None, cv2.CAP_V4L2):
                cap = cv2.VideoCapture(cam_id) if backend is None else cv2.VideoCapture(cam_id, backend)
                if not cap.isOpened():
                    cap.release()
                    continue
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                ok, frame = cap.read()
                if ok and frame is not None:
                    return cap, cam_id
                cap.release()
        return None, None

    def reset_callback(self, msg: Empty):
        """Unlock color decision and enter brief stabilization cooldown."""
        self.locked_decision = None
        self.detection_history.clear()
        self.standby_until_time = time.time() + 1.5
        self.get_logger().info('🔄 Webcam detector reset to standby (1.5s stabilization). Ready for next color!')

    def process_frame(self):
        if self.cap is None or self.should_quit:
            return

        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.read_failures += 1
            if self.read_failures >= 30:
                self.get_logger().error('Webcam stream lost after 30 consecutive read failures.')
                self.should_quit = True
            return
        self.read_failures = 0

        if self.mirror:
            frame = cv2.flip(frame, 1)

        h, w, _ = frame.shape
        debug_frame = frame.copy()

        # Centered Target Box (keeps user's face/background mostly outside ROI)
        roi_y_start = int(h * 0.20)
        roi_y_end = int(h * 0.80)
        roi_x_start = int(w * 0.28)
        roi_x_end = int(w * 0.72)
        roi_area = float((roi_x_end - roi_x_start) * (roi_y_end - roi_y_start))

        roi = frame[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask_red, mask_green, mask_other = compute_color_masks(
            hsv,
            red_h_low=8,
            red_h_high=172,
            red_sat_min=self.red_sat_min,
            red_val_min=70,
            green_h_low=36,
            green_h_high=86,
            green_sat_min=self.green_sat_min,
            green_val_min=60,
            other_sat_min=self.other_sat_min,
            other_val_min=65,
            denoise=True,
        )

        def eval_candidate(mask, name, bgr):
            stats = find_largest_contour_stats(mask, min_pixels=800)
            if stats is None:
                return None, 0.0
            px_count, area, bbox = stats
            fill_ratio = area / roi_area
            return {
                'color': name,
                'bgr': bgr,
                'area': area,
                'fill_ratio': fill_ratio,
                'bbox': bbox,
                'qualified': (fill_ratio >= self.min_fill_ratio and px_count >= int(roi_area * self.min_fill_ratio)),
            }, fill_ratio

        cand_red, red_pct = eval_candidate(mask_red, 'RED', (0, 0, 255))
        cand_green, green_pct = eval_candidate(mask_green, 'GREEN', (0, 200, 0))
        cand_other, other_pct = eval_candidate(mask_other, 'OTHER', (255, 140, 0))

        qualified = [c for c in (cand_red, cand_green, cand_other) if c is not None and c['qualified']]

        frame_color = 'NONE'
        active_candidate = None
        if qualified:
            active_candidate = max(qualified, key=lambda c: c['area'])
            frame_color = active_candidate['color']
            bx, by, bw, bh = active_candidate['bbox']
            cv2.rectangle(
                debug_frame,
                (roi_x_start + bx, roi_y_start + by),
                (roi_x_start + bx + bw, roi_y_start + by + bh),
                active_candidate['bgr'],
                3,
            )
            cv2.putText(
                debug_frame,
                f"{frame_color} ({active_candidate['fill_ratio'] * 100:.0f}%)",
                (roi_x_start + bx, max(65, roi_y_start + by - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                active_candidate['bgr'],
                2,
            )

        # Draw centered ROI target box
        roi_box_color = active_candidate['bgr'] if active_candidate else (255, 255, 0)
        cv2.rectangle(debug_frame, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), roi_box_color, 2)
        cv2.putText(
            debug_frame,
            'Hold Object Here',
            (roi_x_start + 8, roi_y_end + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            roi_box_color,
            2,
        )

        # Temporal debouncing & auto-unlock when object leaves ROI
        is_stabilizing = time.time() < self.standby_until_time
        if is_stabilizing:
            self.detection_history.clear()
            self.empty_frames = 0
            frame_color = 'NONE'
        elif frame_color != 'NONE':
            self.empty_frames = 0
            self.detection_history.append(frame_color)
            if len(self.detection_history) > self.required_confirmations:
                self.detection_history.pop(0)

            if len(self.detection_history) == self.required_confirmations and len(set(self.detection_history)) == 1:
                confirmed = self.detection_history[0]
                if self.locked_decision != confirmed:
                    self.locked_decision = confirmed
                    self.get_logger().info(
                        f'>>> WEBCAM CONFIRMED: [{confirmed}] -> Commanding robot to {PILLAR_DESTINATION_LABEL[confirmed]} <<<'
                    )

                msg_out = String()
                msg_out.data = confirmed
                self.color_pub.publish(msg_out)
        else:
            if self.detection_history:
                self.detection_history.clear()
            if self.locked_decision is not None:
                self.empty_frames += 1
                if self.empty_frames >= 15:  # ~0.5s with no object -> return cleanly to STANDBY
                    self.locked_decision = None
                    self.empty_frames = 0

        # Header & footer status banners
        if is_stabilizing:
            banner_text = 'STABILIZING AT HOME...'
            banner_color = (0, 220, 255)
        elif self.locked_decision:
            banner_text = f'LOCKED: {self.locked_decision} -> {PILLAR_DESTINATION_LABEL[self.locked_decision]}'
            banner_color = (0, 255, 180)
        elif frame_color != 'NONE':
            cnt = sum(1 for c in self.detection_history if c == frame_color)
            banner_text = f'Confirming {frame_color}: {cnt}/{self.required_confirmations}'
            banner_color = active_candidate['bgr'] if active_candidate else (255, 255, 255)
        else:
            banner_text = f'STANDBY | R:{red_pct*100:.0f}% G:{green_pct*100:.0f}% Other:{other_pct*100:.0f}%'
            banner_color = (220, 220, 220)

        cv2.rectangle(debug_frame, (0, 0), (w, 42), (20, 20, 20), -1)
        cv2.putText(debug_frame, banner_text, (10, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.55, banner_color, 2)

        cv2.rectangle(debug_frame, (0, h - 30), (w, h), (20, 20, 20), -1)
        cv2.putText(
            debug_frame,
            '[H] Return Home   [R] Reset Lock   [Q] Quit',
            (10, h - 9),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 200, 200),
            1,
        )

        # Publish annotated debug stream to ROS 2
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'webcam_link'
        self.debug_pub.publish(bgr_to_ros_image(debug_frame, header, bridge=self.bridge))

        # Show interactive OpenCV preview window if enabled and display available
        if self.show_window:
            try:
                cv2.imshow(self.window_name, debug_frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), ord('Q'), 27):  # q or ESC
                    self.get_logger().info('Quit key pressed. Shutting down webcam detector...')
                    self.should_quit = True
                elif key in (ord('h'), ord('H')):
                    self.get_logger().info('🏠 [H] pressed -> Sending Return Home command!')
                    self.home_pub.publish(Empty())
                    self.reset_callback(Empty())
                elif key in (ord('r'), ord('R')):
                    self.get_logger().info('🔄 [R] pressed -> Resetting detector lock!')
                    self.reset_pub.publish(Empty())
                    self.reset_callback(Empty())
            except cv2.error as exc:
                self.get_logger().warn(f'OpenCV window unavailable ({exc}); continuing headless.')
                self.show_window = False

    def cleanup(self):
        """Release camera handle and close OpenCV windows cleanly."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WebcamColorDetector()
    try:
        while rclpy.ok() and not node.should_quit:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

