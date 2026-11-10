#!/usr/bin/env python3
"""
Color Detection Node for Autonomous Navigation
Uses OpenCV to detect whether the object presented in front of the robot is:
- RED
- GREEN
- OTHER (any other color)
Publishes:
- /detected_color (std_msgs/msg/String): Confirmed color classification
- /camera/debug_image (sensor_msgs/msg/Image): Real-time annotated visual feed
"""

import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
import cv2
from color_navigation.vision_utils import (
    bgr_to_ros_image,
    compute_color_masks,
    find_largest_contour_stats,
    ros_image_to_bgr,
)

try:
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    HAS_CV_BRIDGE = False


class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')

        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        # Subscriber to raw front camera stream
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )

        # Publisher for confirmed decision
        self.color_pub = self.create_publisher(
            String,
            '/detected_color',
            10
        )

        # Publisher for OpenCV annotated debug feed
        self.debug_pub = self.create_publisher(
            Image,
            '/camera/debug_image',
            10
        )

        # Subscriber for reset commands
        self.reset_sub = self.create_subscription(
            Empty,
            '/reset_detector',
            self.reset_callback,
            10
        )
        self.home_sub = self.create_subscription(
            Empty,
            '/return_home',
            self.reset_callback,
            10
        )

        # Foreground object proximity criteria (rejects distant background pillars)
        # Distant pillar at 3.5m: width ~65px, area ~11500px, base_y ~301px, px ~11765
        # Close object at 0.9m:   width >=120px, area >=25000px, base_y >=350px, px >=25000
        self.min_pixel_count = 15000
        self.min_contour_area = 14000.0
        self.min_contour_width = 80
        self.min_contour_height = 100
        self.min_base_y = 320

        # Debouncing history: require N consecutive frames of agreement
        self.detection_history = []
        self.required_confirmations = 8
        self.locked_decision = None
        self.standby_until_time = 0.0

        self.get_logger().info('Color Detector Node initialized.')
        self.get_logger().info('Foreground object proximity filter active (rejects distant background pillars).')
        self.get_logger().info('Streaming debug visuals on /camera/debug_image')

    def reset_callback(self, msg: Empty):
        """Reset detection memory and unlock decision for the next object."""
        self.locked_decision = None
        self.detection_history.clear()
        self.standby_until_time = time.time() + 1.2
        self.get_logger().info('Color detector reset to standby (1.2s stabilization window). Ready for new object.')

    def image_callback(self, msg: Image):
        try:
            cv_image = ros_image_to_bgr(msg, bridge=self.bridge)
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        h, w, _ = cv_image.shape
        debug_frame = cv_image.copy()

        # Define Central Region of Interest (ROI)
        roi_y_start = int(h * 0.25)
        roi_y_end = int(h * 0.85)
        roi_x_start = int(w * 0.20)
        roi_x_end = int(w * 0.80)

        # Draw ROI boundary on debug frame (Cyan box)
        cv2.rectangle(
            debug_frame,
            (roi_x_start, roi_y_start),
            (roi_x_end, roi_y_end),
            (255, 255, 0),
            2
        )
        cv2.putText(
            debug_frame,
            'Detection ROI',
            (roi_x_start + 5, roi_y_start - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            1
        )

        roi = cv_image[roi_y_start:roi_y_end, roi_x_start:roi_x_end]
        roi_h, roi_w = roi.shape[:2]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # HSV masks for Red (dual-band wrap), Green, and Other colors
        mask_red, mask_green, mask_other = compute_color_masks(hsv)
        red_pixels = cv2.countNonZero(mask_red)
        green_pixels = cv2.countNonZero(mask_green)
        other_pixels = cv2.countNonZero(mask_other)

        # Foreground object evaluation (Contour & Proximity Filter)
        def evaluate_candidate(mask, color_name, bgr_color):
            stats = find_largest_contour_stats(mask, min_pixels=1000)
            if stats is None:
                return None
            px_count, area, (bx, by, bw, bh) = stats
            base_y = roi_y_start + by + bh
            is_closeup_pillar = (bw >= int(roi_w * 0.90) and bh >= int(roi_h * 0.95))

            is_foreground = (
                not is_closeup_pillar and
                px_count >= self.min_pixel_count and
                area >= self.min_contour_area and
                bw >= self.min_contour_width and
                bh >= self.min_contour_height and
                base_y >= self.min_base_y
            )

            return {
                'color': color_name,
                'bgr': bgr_color,
                'is_foreground': is_foreground,
                'px_count': px_count,
                'area': area,
                'bbox': (bx, by, bw, bh),
                'base_y': base_y
            }

        cand_red = evaluate_candidate(mask_red, 'RED', (0, 0, 255))
        cand_green = evaluate_candidate(mask_green, 'GREEN', (0, 255, 0))
        cand_other = evaluate_candidate(mask_other, 'OTHER', (255, 100, 0))

        all_candidates = [c for c in [cand_red, cand_green, cand_other] if c is not None]
        fg_candidates = [c for c in all_candidates if c['is_foreground']]

        frame_color = 'NONE'
        active_candidate = None

        if fg_candidates:
            # Pick candidate with largest contour area
            active_candidate = max(fg_candidates, key=lambda c: c['area'])
            frame_color = active_candidate['color']
            bx, by, bw, bh = active_candidate['bbox']
            # Draw solid foreground object bounding box
            cv2.rectangle(
                debug_frame,
                (roi_x_start + bx, roi_y_start + by),
                (roi_x_start + bx + bw, roi_y_start + by + bh),
                active_candidate['bgr'],
                3
            )
            cv2.putText(
                debug_frame,
                f"PRESENTED OBJECT: {frame_color} (w={bw}, area={int(active_candidate['area'])})",
                (roi_x_start + bx, roi_y_start + by - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                active_candidate['bgr'],
                2
            )
        else:
            # Draw any background objects with a thin gray box indicating they are ignored
            for c in all_candidates:
                bx, by, bw, bh = c['bbox']
                cv2.rectangle(
                    debug_frame,
                    (roi_x_start + bx, roi_y_start + by),
                    (roi_x_start + bx + bw, roi_y_start + by + bh),
                    (90, 90, 90),
                    1
                )
                cv2.putText(
                    debug_frame,
                    f"Ignored Background {c['color']} (w={bw}<{self.min_contour_width})",
                    (roi_x_start + bx, roi_y_start + by - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (120, 120, 120),
                    1
                )

        now_sec = time.time()
        is_stabilizing = (now_sec < self.standby_until_time)
        webcam_override = (self.count_publishers('/webcam_active') > 0)

        # Debouncing filter
        if webcam_override or is_stabilizing:
            self.detection_history.clear()
            if webcam_override:
                self.locked_decision = None
            frame_color = 'NONE'
        elif frame_color != 'NONE':
            self.detection_history.append(frame_color)
            if len(self.detection_history) > self.required_confirmations:
                self.detection_history.pop(0)

            if len(self.detection_history) == self.required_confirmations and len(set(self.detection_history)) == 1:
                confirmed = self.detection_history[0]
                if self.locked_decision != confirmed:
                    self.locked_decision = confirmed
                    self.get_logger().info(f'>>> FOREGROUND COLOR CONFIRMED: [{confirmed}] <<<')

                msg_out = String()
                msg_out.data = confirmed
                self.color_pub.publish(msg_out)
        else:
            # Clear history if no foreground object is present
            if self.detection_history:
                self.detection_history.clear()

        # Visual overlay header banner
        if webcam_override:
            status_text = "YIELDED TO LAPTOP WEBCAM"
        elif is_stabilizing:
            status_text = "STABILIZING AT HOME..."
        elif self.locked_decision:
            status_text = f"LOCKED: {self.locked_decision}"
        else:
            status_text = f"Detecting: {frame_color}"
        cv2.rectangle(debug_frame, (0, 0), (w, 40), (20, 20, 20), -1)
        cv2.putText(
            debug_frame,
            f"Color Detection: {status_text} | Foreground: {len(fg_candidates)} | R:{red_pixels} G:{green_pixels} Other:{other_pixels}",
            (10, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            active_candidate['bgr'] if active_candidate else (180, 180, 180),
            2
        )

        # Publish debug visualization
        debug_msg = bgr_to_ros_image(debug_frame, msg.header, bridge=self.bridge)
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
