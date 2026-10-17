#!/usr/bin/env python3
"""Publish this robot's map-frame pose at a fixed rate on shared_pose.

The other robots' RobotAvoidanceLayer reads it. AMCL's own amcl_pose is
published only after a filter update (every update_min_d / update_min_a of
motion), so a parked robot would go silent and, after the layer's
max_pose_age_s, vanish from the other robots' costmaps: exactly the parked
case where the layer matters most. This node looks up map -> base_link
(AMCL's map -> odom composed with the EKF's odom -> base_link) at rate_hz
instead and republishes it as geometry_msgs/PoseWithCovarianceStamped.
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SharedPosePublisher(Node):

    def __init__(self):
        super().__init__("shared_pose_publisher")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self._global = self.get_parameter("global_frame").value
        self._base = self.get_parameter("base_frame").value
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._pub = self.create_publisher(PoseWithCovarianceStamped, "shared_pose", 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._tick)

    def _tick(self):
        try:
            t = self._buffer.lookup_transform(self._global, self._base, Time())
        except TransformException:
            return   # not localized yet; the layer treats silence as "unknown"
        msg = PoseWithCovarianceStamped()
        msg.header = t.header
        msg.pose.pose.position.x = t.transform.translation.x
        msg.pose.pose.position.y = t.transform.translation.y
        msg.pose.pose.position.z = t.transform.translation.z
        msg.pose.pose.orientation = t.transform.rotation
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = SharedPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
