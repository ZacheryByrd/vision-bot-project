"""ROS 2 node connecting one robot to the fleet API; run one per robot, in its namespace.

Parameters: robot_id (the robot's fleet serial number), robot_name (default:
the namespace), api_base_url (default: env FLEET_API_BASE_URL), and the
telemetry/poll periods. Credentials come from env FLEET_USER and
FLEET_PASSWORD, never from parameters.

Threads: HTTP calls block, so BridgeCore.step runs on its own fleet thread.
Every rclpy call stays on the executor thread: the fleet thread only queues
Nav2 sends and cancels, and a 10 Hz timer carries them out.
"""

import logging
import math
import os
import threading
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from fleet_bridge.client import FleetClient, UrllibSession
from fleet_bridge.core import BridgeCore, yaw_from_quaternion

NAV2_OUTCOME = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
                GoalStatus.STATUS_CANCELED: "canceled"}


class BridgeNode(Node):
    def __init__(self):
        super().__init__("fleet_bridge")
        robot_id = self.declare_parameter("robot_id", "").value
        if not robot_id:
            raise SystemExit("fleet_bridge: set the robot_id parameter (the fleet serial number)")
        name = self.declare_parameter("robot_name", "").value or self.get_namespace().strip("/")
        base_url = self.declare_parameter(
            "api_base_url",
            os.environ.get("FLEET_API_BASE_URL", "http://localhost:8000/api/v1")).value
        telemetry_s = self.declare_parameter("telemetry_period_s", 1.0).value
        poll_s = self.declare_parameter("poll_period_s", 1.5).value
        user, password = os.environ.get("FLEET_USER"), os.environ.get("FLEET_PASSWORD")
        if not user or not password:
            raise SystemExit("fleet_bridge: set FLEET_USER and FLEET_PASSWORD (see .env.example)")

        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._outbox = deque()          # ("send", Goal) / ("cancel", goal_id) from the fleet thread
        self._handles = {}              # goal_id -> ClientGoalHandle (executor thread only)
        self._cancel_on_accept = set()
        client = FleetClient(base_url, user, password, UrllibSession())
        self.core = BridgeCore(client, robot_id, name, send=self._queue_send,
                               cancel=self._queue_cancel, telemetry_period_s=telemetry_s,
                               poll_period_s=poll_s)
        self.create_subscription(PoseWithCovarianceStamped, "shared_pose", self._on_pose, 10)
        self.create_timer(0.1, self._drain_outbox)
        threading.Thread(target=self._fleet_loop, daemon=True).start()
        self.get_logger().info(f"bridging {self.get_namespace()} as {robot_id} to {base_url}")

    # -- fleet thread -------------------------------------------------------
    def _fleet_loop(self):
        while rclpy.ok():
            try:
                self.core.step(time.monotonic())
            except Exception as err:      # never let the bridge thread die
                self.get_logger().error(f"bridge step failed: {type(err).__name__}: {err}")
            time.sleep(0.1)

    def _queue_send(self, goal):
        self._outbox.append(("send", goal))

    def _queue_cancel(self, goal_id):
        self._outbox.append(("cancel", goal_id))

    # -- executor thread -----------------------------------------------------
    def _on_pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.core.update_pose(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _drain_outbox(self):
        while self._outbox:
            kind, item = self._outbox.popleft()
            if kind == "send":
                self._send(item)
            else:
                self._cancel(item)

    def _send(self, goal):
        if not self._nav.server_is_ready():
            self.get_logger().warning(f"goal {goal.goal_id}: navigate_to_pose is not available")
            self.core.on_nav_result(goal.goal_id, "rejected")
            return
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = goal.x, goal.y
        q = yaw_to_quaternion(goal.yaw)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        self.get_logger().info(f"goal {goal.goal_id}: to ({goal.x:.2f}, {goal.y:.2f})")
        future = self._nav.send_goal_async(msg)
        future.add_done_callback(lambda f, gid=goal.goal_id: self._on_goal_response(gid, f))

    def _on_goal_response(self, goal_id, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning(f"goal {goal_id}: rejected by Nav2")
            self._cancel_on_accept.discard(goal_id)
            self.core.on_nav_result(goal_id, "rejected")
            return
        self._handles[goal_id] = handle
        self.core.on_nav_result(goal_id, "accepted")
        handle.get_result_async().add_done_callback(
            lambda f, gid=goal_id: self._on_result(gid, f))
        if goal_id in self._cancel_on_accept:
            self._cancel(goal_id)

    def _on_result(self, goal_id, future):
        self._handles.pop(goal_id, None)
        self._cancel_on_accept.discard(goal_id)
        status = future.result().status
        outcome = NAV2_OUTCOME.get(status, "aborted")
        self.get_logger().info(f"goal {goal_id}: {outcome}")
        self.core.on_nav_result(goal_id, outcome)

    def _cancel(self, goal_id):
        handle = self._handles.get(goal_id)
        if handle is None:                # not accepted yet: cancel once it is
            self._cancel_on_accept.add(goal_id)
            return
        self._cancel_on_accept.discard(goal_id)
        handle.cancel_goal_async()


def yaw_to_quaternion(yaw):
    """(z, w) of a rotation about the vertical axis."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main():
    logging.basicConfig(level=logging.INFO,
                        format="[fleet_bridge] %(levelname)s %(name)s: %(message)s")
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
