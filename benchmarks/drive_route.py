#!/usr/bin/env python3
"""Drive the robot along a named route from benchmarks/route.py.

Publishes on cmd_vel_teleop (the teleop input of twist_mux) and steers from
ground_truth/pose, so it is keyboard teleop done by a script. Used for the
mapping run and, through RouteDriver, by odom_drift.py.

Run inside the container with nav_sim.launch.py or slam.launch.py up:
    python3 benchmarks/drive_route.py --route explore
"""

import argparse
import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from route import ROUTES, pose_from_odometry, reached, steer


class RouteDriver:
    """Follows waypoints from inside an existing node (20 Hz, node clock)."""

    def __init__(self, node, waypoints, cmd_topic="cmd_vel_teleop",
                 truth_topic="ground_truth/pose", waypoint_timeout_s=90.0):
        self._node = node
        self._waypoints = list(waypoints)
        self._timeout = waypoint_timeout_s
        self._index = 0
        self._leg_start = None
        self.pose = None
        self.done = False
        self.failed = None
        self._pub = node.create_publisher(Twist, cmd_topic, 10)
        node.create_subscription(Odometry, truth_topic, self._on_truth, qos_profile_sensor_data)
        node.create_timer(0.05, self._tick)

    def _on_truth(self, msg):
        self.pose = pose_from_odometry(msg)

    def _stop(self, failure=None):
        self.failed = failure
        self.done = True
        self._pub.publish(Twist())

    def _tick(self):
        if self.done or self.pose is None:
            return
        now = self._node.get_clock().now().nanoseconds * 1e-9
        if self._leg_start is None:
            self._leg_start = now
        goal = self._waypoints[self._index]
        if reached(self.pose, goal):
            self._index += 1
            self._leg_start = now
            if self._index == len(self._waypoints):
                self._stop()
                return
            goal = self._waypoints[self._index]
        elif now - self._leg_start > self._timeout:
            self._stop(f"waypoint {self._index} {goal} not reached within "
                       f"{self._timeout} s of sim time")
            return
        msg = Twist()
        msg.linear.x, msg.angular.z = steer(self.pose, goal)
        self._pub.publish(msg)


def make_node(name):
    """A node on Gazebo's clock, so timers and timeouts run in sim time."""
    return rclpy.create_node(
        name, parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--route", choices=sorted(ROUTES), required=True)
    parser.add_argument("--cmd-topic", default="cmd_vel_teleop")
    args = parser.parse_args()

    rclpy.init()
    node = make_node("drive_route")
    driver = RouteDriver(node, ROUTES[args.route], cmd_topic=args.cmd_topic)
    start = None
    while rclpy.ok() and not driver.done:
        rclpy.spin_once(node, timeout_sec=0.05)
        if start is None and driver.pose is not None:
            start = node.get_clock().now().nanoseconds * 1e-9
    elapsed = node.get_clock().now().nanoseconds * 1e-9 - (start or 0.0)
    node.destroy_node()
    rclpy.shutdown()
    if driver.failed:
        print(f"drive_route {args.route}: FAILED: {driver.failed}")
        sys.exit(1)
    print(f"drive_route {args.route}: done in {elapsed:.1f} s of sim time")


if __name__ == "__main__":
    main()
