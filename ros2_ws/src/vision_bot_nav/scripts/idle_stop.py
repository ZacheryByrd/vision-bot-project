#!/usr/bin/env python3
"""Publish a zero velocity on cmd_vel_idle, twist_mux's lowest-priority input.

twist_mux forwards a command only when one of its inputs publishes, and the
Gazebo diff-drive plugin keeps executing its last command forever. Without
this node, once every real source stops (teleop released, line follow off,
Nav2 gone) the robot keeps driving on whatever it was last told. With it,
this zero input takes over as soon as every higher-priority input has timed
out, so "no active source" means "stopped".
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class IdleStop(Node):

    def __init__(self):
        super().__init__("idle_stop")
        self.declare_parameter("rate_hz", 10.0)
        self._pub = self.create_publisher(Twist, "cmd_vel_idle", 10)
        self.create_timer(1.0 / self.get_parameter("rate_hz").value, self._tick)

    def _tick(self):
        self._pub.publish(Twist())


def main():
    rclpy.init()
    node = IdleStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
