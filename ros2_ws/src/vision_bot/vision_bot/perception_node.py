#!/usr/bin/env python3
"""
perception_node
================
Owns the camera. Runs OpenCV. Publishes a detection message.

Does NOT know anything about motors, /cmd_vel, or how the robot moves.
That separation is the whole point of the architecture: perception and
control are independent nodes that only talk to each other over topics,
so either one can be tested, swapped, or reused on its own.

Detection strategy (default): HSV color-threshold + largest-contour
tracking. This is intentionally simple to get the pipeline working
end-to-end first. Swap `_detect` for a DNN model (OpenCV's `cv2.dnn`
with a MobileNet-SSD, or a TFLite model) once the plumbing is proven --
the rest of the node (message publishing, params, debug image) doesn't
need to change.

Published topics:
    /vision_bot/detection   (std_msgs/Float32MultiArray)
        [0] detected      -> 1.0 if a target is in frame, else 0.0
        [1] offset_x      -> normalized horizontal offset of target center
                              from image center, range [-1.0, 1.0]
                              (negative = target is left of center)
        [2] offset_y      -> normalized vertical offset, same convention
        [3] area_fraction -> target bounding-box area / image area,
                              range [0.0, 1.0] -- used as a rough distance proxy
        [4] confidence    -> placeholder for a future model's confidence score;
                              1.0 for the contour-based detector when found

    /vision_bot/debug_image (sensor_msgs/Image)
        Annotated frame (bounding box + centroid drawn) for the dashboard
        and for debugging with `ros2 run rqt_image_view rqt_image_view`.

Subscribed topics:
    /camera/image_raw (sensor_msgs/Image) -- from Gazebo's camera plugin
    in sim, or from a v4l2_camera/usb_cam node on hardware. Configurable
    via the `image_topic` parameter so sim and hardware launch files can
    point at different sources without touching this file.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge


def detect_target(frame: np.ndarray, *, hue_low, hue_high, sat_low, val_low, min_area):
    """HSV threshold + largest-contour tracking.

    Pulled out of PerceptionNode so it can be unit tested directly against
    synthetic images -- no rclpy node, no camera, just OpenCV. See
    test/test_perception_logic.py.

    Returns (detected: bool, offset_x, offset_y, area_fraction, debug_frame).
    offset_x/offset_y are normalized to [-1, 1]; 0 means dead-center.
    """
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = np.array([hue_low, sat_low, val_low])
    upper = np.array([hue_high, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    debug_frame = frame.copy()

    if not contours:
        return False, 0.0, 0.0, 0.0, debug_frame

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return False, 0.0, 0.0, 0.0, debug_frame

    x, y, bw, bh = cv2.boundingRect(largest)
    cx, cy = x + bw / 2.0, y + bh / 2.0

    offset_x = (cx - w / 2.0) / (w / 2.0)
    offset_y = (cy - h / 2.0) / (h / 2.0)
    area_fraction = min(1.0, area / float(w * h))

    cv2.rectangle(debug_frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
    cv2.circle(debug_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
    cv2.line(debug_frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

    return True, offset_x, offset_y, area_fraction, debug_frame


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")

        # --- Parameters (override via launch file or `ros2 param set`) ---
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("target_hue_low", 0)      # default: red target
        self.declare_parameter("target_hue_high", 10)
        self.declare_parameter("target_sat_low", 120)
        self.declare_parameter("target_val_low", 80)
        self.declare_parameter("min_contour_area", 200)   # px^2, filters noise
        self.declare_parameter("publish_debug_image", True)

        image_topic = self.get_parameter("image_topic").value

        self.bridge = CvBridge()
        self.detection_pub = self.create_publisher(
            Float32MultiArray, "/vision_bot/detection", 10
        )
        self.debug_pub = self.create_publisher(
            Image, "/vision_bot/debug_image", 10
        )
        self.image_sub = self.create_subscription(
            Image, image_topic, self._on_image, 10
        )

        self.get_logger().info(
            f"perception_node up, subscribed to '{image_topic}', "
            f"publishing detections on '/vision_bot/detection'"
        )

    def _on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        detected, offset_x, offset_y, area_fraction, debug_frame = self._detect(frame)

        out = Float32MultiArray()
        out.data = [
            1.0 if detected else 0.0,
            float(offset_x),
            float(offset_y),
            float(area_fraction),
            1.0 if detected else 0.0,  # confidence placeholder
        ]
        self.detection_pub.publish(out)

        if self.get_parameter("publish_debug_image").value:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def _detect(self, frame: np.ndarray):
        """HSV threshold + largest-contour tracking.

        Returns (detected: bool, offset_x, offset_y, area_fraction, debug_frame).
        offset_x/offset_y are normalized to [-1, 1]; 0 means dead-center.
        """
        return detect_target(
            frame,
            hue_low=self.get_parameter("target_hue_low").value,
            hue_high=self.get_parameter("target_hue_high").value,
            sat_low=self.get_parameter("target_sat_low").value,
            val_low=self.get_parameter("target_val_low").value,
            min_area=self.get_parameter("min_contour_area").value,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
