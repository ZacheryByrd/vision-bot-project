#!/usr/bin/env python3
"""drone_flight: flies the simulated drone along seeded random waypoints.

Spawns the `drone` model if it is not in the world, then sets its pose at
rate_hz through /gazebo/set_entity_state (the gazebo_ros_state plugin that
nav_world.world loads) and publishes the same pose as ground truth:
    /drone/ground_truth           geometry_msgs/PoseStamped, frame map
    /drone/ground_truth_velocity  geometry_msgs/Vector3Stamped, frame map
The tracker never reads these; they are for scoring.

Parameters:
    seed    (int, default -1)   -1 draws a fresh seed; logged as "drone_flight seed=<n>"
    bounds  (double[4])         x_min, x_max, y_min, y_max; empty = the nav_world map extent
    rate_hz (double, default 30)
Run with use_sim_time:=true so the flight follows simulation time.
"""

import os
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, SpawnEntity
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from rclpy.node import Node

from vision_bot_drone_sim.flight_path import make_waypoints, pose_at


def map_bounds(yaml_path):
    """(x_min, x_max, y_min, y_max) of a map_server map: origin plus image size times resolution."""
    meta = yaml.safe_load(Path(yaml_path).read_text())
    with open(Path(yaml_path).parent / meta["image"], "rb") as f:
        f.readline()  # P5
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = (int(v) for v in line.split())
    x0, y0 = meta["origin"][0], meta["origin"][1]
    res = meta["resolution"]
    return (x0, x0 + width * res, y0, y0 + height * res)


class DroneFlight(Node):
    def __init__(self):
        super().__init__("drone_flight")
        seed = self.declare_parameter("seed", -1).value
        bounds = list(self.declare_parameter("bounds", [0.0]).value)
        self.rate_hz = self.declare_parameter("rate_hz", 30.0).value
        if seed < 0:
            seed = int.from_bytes(os.urandom(4), "little") & 0x7FFFFFFF
        if len(bounds) != 4:
            nav = get_package_share_directory("vision_bot_nav")
            bounds = map_bounds(os.path.join(nav, "maps", "nav_world.yaml"))
        self.seed = seed
        self.waypoints = make_waypoints(seed, tuple(bounds))
        self.get_logger().info(
            f"drone_flight seed={seed} bounds={[round(b, 2) for b in bounds]} "
            f"first waypoint=({self.waypoints[0].x:.2f}, {self.waypoints[0].y:.2f}, "
            f"{self.waypoints[0].z:.2f})")

        self.truth_pub = self.create_publisher(PoseStamped, "/drone/ground_truth", 10)
        self.vel_pub = self.create_publisher(Vector3Stamped, "/drone/ground_truth_velocity", 10)
        self.set_state = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.spawn = self.create_client(SpawnEntity, "/spawn_entity")
        self.t0 = None
        self.spawned = False
        self.create_timer(1.0 / self.rate_hz, self.tick)

    def spawn_drone(self):
        model = os.path.join(get_package_share_directory("vision_bot_drone_sim"),
                             "models", "drone", "model.sdf")
        req = SpawnEntity.Request(name="drone", xml=Path(model).read_text())
        first = self.waypoints[0]
        req.initial_pose.position.x, req.initial_pose.position.y = first.x, first.y
        req.initial_pose.position.z = first.z
        # Already present (a restarted flight node) fails harmlessly; the pose
        # updates below move whichever drone is there.
        self.spawn.call_async(req)
        self.spawned = True

    def tick(self):
        if not self.spawned:
            if not (self.spawn.service_is_ready() and self.set_state.service_is_ready()):
                return  # Gazebo still starting
            self.spawn_drone()
            return
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
        t = (now - self.t0).nanoseconds * 1e-9
        (x, y, z), (vx, vy, vz) = pose_at(self.waypoints, t)

        state = EntityState(name="drone", reference_frame="world")
        state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
        state.pose.orientation.w = 1.0
        self.set_state.call_async(SetEntityState.Request(state=state))

        truth = PoseStamped()
        truth.header.stamp = now.to_msg()
        truth.header.frame_id = "map"
        truth.pose = state.pose
        self.truth_pub.publish(truth)
        vel = Vector3Stamped()
        vel.header = truth.header
        vel.vector.x, vel.vector.y, vel.vector.z = vx, vy, vz
        self.vel_pub.publish(vel)


def main(args=None):
    rclpy.init(args=args)
    node = DroneFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
