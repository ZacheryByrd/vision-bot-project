#!/usr/bin/env python3
"""
perception_node
================
Owns the camera. Runs OpenCV. Publishes a detection message.

Does NOT know anything about motors, /cmd_vel, or how the robot moves.
That separation is the whole point of the architecture: perception and
control are independent nodes that only talk to each other over topics,
so either one can be tested, swapped, or reused on its own.

Detection strategy: selectable via the `detector_backend` parameter.
    "hsv" (default) -- HSV color-threshold + largest-contour tracking
        (detect_target). Simple, fast, no model file, but only finds
        "the biggest blob of roughly this color" -- it has no concept
        of what a red box or a yellow line actually is.
    "dnn" -- a pretrained MobileNet-SSD (Caffe, via cv2.dnn) that
        recognizes real object categories (dnn_detect_target). See
        dnn_follow_launch.py, which uses this to detect an actual
        "person" class rather than a hand-picked color.
Either way the rest of the node (message publishing, params, debug
image) doesn't change -- that's the point of keeping both behind the
same _detect() -> (detected, offset_x, offset_y, area_fraction,
confidence, debug_frame) interface.

Published topics:
    /vision_bot/detection   (std_msgs/Float32MultiArray)
        [0] detected      -> 1.0 if a target is in frame, else 0.0
        [1] offset_x      -> normalized horizontal offset of target center
                              from image center, range [-1.0, 1.0]
                              (negative = target is left of center)
        [2] offset_y      -> normalized vertical offset, same convention
        [3] area_fraction -> target bounding-box area / image area,
                              range [0.0, 1.0] -- used as a rough distance proxy
        [4] confidence    -> 1.0 when detected for the HSV backend (it has no
                              real notion of confidence); a real model score
                              in [0.0, 1.0] for the DNN backend

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

    Returns (detected: bool, offset_x, offset_y, area_fraction, confidence,
    debug_frame). offset_x/offset_y are normalized to [-1, 1]; 0 means
    dead-center. This detector has no real notion of confidence (a contour
    either passes the threshold or doesn't), so confidence is always 1.0
    when detected and 0.0 otherwise -- contrast with dnn_detect_target,
    where it's a real model score.
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
        return False, 0.0, 0.0, 0.0, 0.0, debug_frame

    largest = max(contours, key=cv2.contourArea)
    area = cv2.contourArea(largest)
    if area < min_area:
        return False, 0.0, 0.0, 0.0, 0.0, debug_frame

    x, y, bw, bh = cv2.boundingRect(largest)
    cx, cy = x + bw / 2.0, y + bh / 2.0

    offset_x = (cx - w / 2.0) / (w / 2.0)
    offset_y = (cy - h / 2.0) / (h / 2.0)
    area_fraction = min(1.0, area / float(w * h))

    cv2.rectangle(debug_frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
    cv2.circle(debug_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
    cv2.line(debug_frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

    return True, offset_x, offset_y, area_fraction, 1.0, debug_frame


def dnn_detect_target(frame: np.ndarray, *, net, target_class_id, confidence_threshold):
    """MobileNet-SSD (Caffe, via cv2.dnn) object detection.

    Same return shape as detect_target -- (detected, offset_x, offset_y,
    area_fraction, confidence, debug_frame) -- so PerceptionNode can swap
    between the two with no other code changes, and motor_control_node
    never needs to know which one is running.

    Confidence here is a real model score, not a placeholder. Picks the
    highest-confidence detection of `target_class_id` in frame, ignoring
    every other class the model knows about.

    `net` is a pre-loaded cv2.dnn_Net (see PerceptionNode.__init__ --
    loading it fresh on every frame would be far too slow to run at
    camera frame rate).
    """
    h, w = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
    net.setInput(blob)
    detections = net.forward()

    debug_frame = frame.copy()

    best_confidence = 0.0
    best_box = None
    for i in range(detections.shape[2]):
        confidence = float(detections[0, 0, i, 2])
        class_id = int(detections[0, 0, i, 1])
        if class_id == target_class_id and confidence >= confidence_threshold:
            if confidence > best_confidence:
                best_confidence = confidence
                best_box = detections[0, 0, i, 3:7]

    if best_box is None:
        return False, 0.0, 0.0, 0.0, 0.0, debug_frame

    x1, y1, x2, y2 = (best_box * [w, h, w, h])
    bw, bh = x2 - x1, y2 - y1
    cx, cy = x1 + bw / 2.0, y1 + bh / 2.0

    offset_x = (cx - w / 2.0) / (w / 2.0)
    offset_y = (cy - h / 2.0) / (h / 2.0)
    area_fraction = min(1.0, (bw * bh) / float(w * h))

    cv2.rectangle(debug_frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
    cv2.circle(debug_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
    cv2.line(debug_frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)

    return True, offset_x, offset_y, area_fraction, best_confidence, debug_frame


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

        # detector_backend: "hsv" (default, detect_target above) or "dnn"
        # (dnn_detect_target, a pretrained MobileNet-SSD via cv2.dnn) --
        # see dnn_follow_launch.py for the demo that uses the latter.
        self.declare_parameter("detector_backend", "hsv")
        self.declare_parameter("dnn_prototxt_path", "")
        self.declare_parameter("dnn_model_path", "")
        self.declare_parameter("dnn_target_class_id", 15)     # 15 = "person" in the VOC 20 classes
        self.declare_parameter("dnn_confidence_threshold", 0.5)

        image_topic = self.get_parameter("image_topic").value

        self.dnn_net = None
        if self.get_parameter("detector_backend").value == "dnn":
            prototxt = self.get_parameter("dnn_prototxt_path").value
            model = self.get_parameter("dnn_model_path").value
            self.dnn_net = cv2.dnn.readNetFromCaffe(prototxt, model)
            self.get_logger().info(f"loaded DNN model: {prototxt} / {model}")

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
        detected, offset_x, offset_y, area_fraction, confidence, debug_frame = self._detect(frame)

        out = Float32MultiArray()
        out.data = [
            1.0 if detected else 0.0,
            float(offset_x),
            float(offset_y),
            float(area_fraction),
            float(confidence),
        ]
        self.detection_pub.publish(out)

        if self.get_parameter("publish_debug_image").value:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)

    def _detect(self, frame: np.ndarray):
        """Dispatches to whichever backend `detector_backend` selects.

        Returns (detected: bool, offset_x, offset_y, area_fraction,
        confidence, debug_frame). offset_x/offset_y are normalized to
        [-1, 1]; 0 means dead-center.
        """
        if self.dnn_net is not None:
            return dnn_detect_target(
                frame,
                net=self.dnn_net,
                target_class_id=self.get_parameter("dnn_target_class_id").value,
                confidence_threshold=self.get_parameter("dnn_confidence_threshold").value,
            )
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
