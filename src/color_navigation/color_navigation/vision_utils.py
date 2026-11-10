#!/usr/bin/env python3
"""
Shared OpenCV and ROS 2 Image helpers for Gazebo and laptop webcam color detectors.
Provides ROS Image <-> OpenCV BGR conversion and parameterized HSV masking for:
- RED   (dual-band hue wrap around 0 and 180)
- GREEN
- OTHER (saturated colors outside Red and Green)
"""

import cv2
import numpy as np
from sensor_msgs.msg import Image


def ros_image_to_bgr(msg: Image, bridge=None) -> np.ndarray:
    """Convert ROS sensor_msgs/Image to an OpenCV BGR uint8 array."""
    if bridge is not None:
        return bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
    if msg.encoding in ('rgb8', 'RGB8'):
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img.copy()


def bgr_to_ros_image(cv_image: np.ndarray, header, bridge=None) -> Image:
    """Convert an OpenCV BGR uint8 array to ROS sensor_msgs/Image."""
    if bridge is not None:
        ros_msg = bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        ros_msg.header = header
        return ros_msg

    ros_msg = Image()
    ros_msg.header = header
    ros_msg.height, ros_msg.width, _ = cv_image.shape
    ros_msg.encoding = 'bgr8'
    ros_msg.is_bigendian = 0
    ros_msg.step = ros_msg.width * 3
    ros_msg.data = cv_image.tobytes()
    return ros_msg


def compute_color_masks(
    hsv_roi: np.ndarray,
    *,
    red_h_low: int = 10,
    red_h_high: int = 170,
    red_sat_min: int = 70,
    red_val_min: int = 50,
    green_h_low: int = 35,
    green_h_high: int = 85,
    green_sat_min: int = 70,
    green_val_min: int = 50,
    other_sat_min: int = 60,
    other_val_min: int = 50,
    denoise: bool = False,
):
    """Build (mask_red, mask_green, mask_other) binary masks from an HSV ROI."""
    mask_red_1 = cv2.inRange(
        hsv_roi,
        np.array([0, red_sat_min, red_val_min]),
        np.array([red_h_low, 255, 255]),
    )
    mask_red_2 = cv2.inRange(
        hsv_roi,
        np.array([red_h_high, red_sat_min, red_val_min]),
        np.array([180, 255, 255]),
    )
    mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)

    mask_green = cv2.inRange(
        hsv_roi,
        np.array([green_h_low, green_sat_min, green_val_min]),
        np.array([green_h_high, 255, 255]),
    )

    mask_any = cv2.inRange(
        hsv_roi,
        np.array([0, other_sat_min, other_val_min]),
        np.array([180, 255, 255]),
    )
    mask_other = cv2.bitwise_and(
        mask_any,
        cv2.bitwise_not(cv2.bitwise_or(mask_red, mask_green)),
    )

    if denoise:
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_other = cv2.morphologyEx(mask_other, cv2.MORPH_OPEN, kernel)
        mask_other = cv2.morphologyEx(mask_other, cv2.MORPH_CLOSE, kernel)

    return mask_red, mask_green, mask_other


def find_largest_contour_stats(mask: np.ndarray, min_pixels: int = 1000):
    """Return (px_count, area, (bx, by, bw, bh)) for the largest contour or None."""
    px_count = cv2.countNonZero(mask)
    if px_count < min_pixels:
        return None
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(largest))
    bbox = cv2.boundingRect(largest)
    return px_count, area, bbox

