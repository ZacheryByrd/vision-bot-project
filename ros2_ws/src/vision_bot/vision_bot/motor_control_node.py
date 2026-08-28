#!/usr/bin/env python3
"""
motor_control_node
===================
Owns motor/velocity output. Turns a detection message into a Twist
command. Does NOT know anything about cameras, OpenCV, or pixels --
it only ever sees a normalized offset and an area fraction. That's
what makes this node hardware-agnostic: in sim, /cmd_vel drives
Gazebo's diff-drive plugin; on hardware, a separate small node
(gpio_motor_driver.py) subscribes to the same /cmd_vel topic and
drives the real motor driver. This node never changes between the two.

Subscribed topics:
    /vision_bot/detection (std_msgs/Float32MultiArray)
        See perception_node.py for the field layout.

Published topics:
    /cmd_vel (geometry_msgs/Twist)

Control law (deliberately simple -- a proportional controller):
    angular.z = -Kp_angular * offset_x           (turn toward the target)
    linear.x  = base_speed * (1 - |offset_x|)     (slow down while turning)
                reduced further as area_fraction approaches
                `stop_area_fraction`, and held at 0 once the target is
                that big in-frame -- this is what stops the rover from
                driving straight into the target instead of just slowing
                down near it (slowing alone was tried first and still let
                it creep forward until it collided).

    If no detection for `lost_timeout` seconds, stop (or optionally spin
    slowly to search -- toggle via the `search_when_lost` parameter).
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist


class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_control_node")

        self.declare_parameter("base_linear_speed", 0.15)   # m/s
        self.declare_parameter("angular_gain", 1.0)          # rad/s per unit offset
        self.declare_parameter("offset_deadband", 0.05)      # ignore tiny offsets
        self.declare_parameter("lost_timeout_sec", 0.75)
        self.declare_parameter("search_when_lost", True)
        self.declare_parameter("search_angular_speed", 0.3)
        self.declare_parameter("stop_area_fraction", 0.15)   # halt approach once target is this big in-frame

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.detection_sub = self.create_subscription(
            Float32MultiArray, "/vision_bot/detection", self._on_detection, 10
        )

        self._last_detection_time = 0.0
        self._last_seen_direction = 1.0  # for search behavior: which way to spin

        # Watchdog timer: if perception goes quiet, stop the robot rather
        # than keep executing a stale command.
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info("motor_control_node up, publishing on /cmd_vel")

    def _on_detection(self, msg: Float32MultiArray):
        detected, offset_x, offset_y, area_fraction, confidence = msg.data
        self._last_detection_time = time.time()

        cmd = Twist()

        if detected < 0.5:
            # No target this frame; let the watchdog decide what to do.
            return

        deadband = self.get_parameter("offset_deadband").value
        if abs(offset_x) < deadband:
            offset_x = 0.0
        else:
            self._last_seen_direction = 1.0 if offset_x > 0 else -1.0

        angular_gain = self.get_parameter("angular_gain").value
        base_speed = self.get_parameter("base_linear_speed").value

        cmd.angular.z = -angular_gain * offset_x

        stop_area_fraction = self.get_parameter("stop_area_fraction").value
        if area_fraction >= stop_area_fraction:
            # Close enough -- hold position (still allowed to rotate to
            # stay centered) instead of creeping forward into the target.
            cmd.linear.x = 0.0
        else:
            # Slow down proportionally to how far off-center we are, and
            # further as the target grows toward the stop threshold.
            proximity_factor = max(0.2, 1.0 - area_fraction / stop_area_fraction)
            cmd.linear.x = base_speed * (1.0 - min(1.0, abs(offset_x))) * proximity_factor

        self.cmd_pub.publish(cmd)

    def _watchdog(self):
        elapsed = time.time() - self._last_detection_time
        timeout = self.get_parameter("lost_timeout_sec").value
        if elapsed <= timeout:
            return  # still receiving fresh detections, nothing to do here

        cmd = Twist()
        if self.get_parameter("search_when_lost").value:
            cmd.angular.z = (
                self.get_parameter("search_angular_speed").value
                * self._last_seen_direction
            )
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
