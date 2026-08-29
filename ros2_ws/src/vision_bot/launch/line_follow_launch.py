"""
line_follow_launch.py
======================
The line-following variant of sim_launch.py: same nodes (perception_node,
motor_control_node), same topics, different world (a yellow octagon track
instead of a red box) and different tuning. Thin wrapper -- includes
sim_launch.py with overridden launch arguments rather than duplicating
the whole node graph.

Run with:
    ros2 launch vision_bot line_follow_launch.py

Tuning notes vs. the red-box demo:
    - target_hue_low/high ~ 25-35: yellow in OpenCV's 0-179 hue range
      (pure yellow is ~30). Doesn't overlap the red demo's 0-10 range.
    - stop_area_fraction: 1.0 -- effectively disables the "stop when
      close" behavior from the red-box demo. area_fraction is capped at
      1.0 (see perception_node.py), so this threshold is in practice
      unreachable; the line always looks like it needs following, never
      like something to stop in front of. Line-following wants continuous
      forward motion, not an approach-and-halt.
    - angular_gain: higher than the red-box demo's 1.0 -- the octagon's
      45-degree turns need a punchier correction than tracking a
      slow-moving/stationary object does.
    - base_linear_speed: a bit slower than the red-box demo, trading lap
      time for turning margin at the corners.
    - spawn_x/y/yaw: the middle of the octagon's first segment, facing
      along it (see worlds/line_track.world's comment for how the
      geometry was derived), instead of the red-box demo's world-origin
      default.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "sim_launch.py")
        ),
        launch_arguments={
            "world": "line_track.world",
            "target_hue_low": "25",
            "target_hue_high": "35",
            "target_sat_low": "120",
            "target_val_low": "80",
            "min_contour_area": "150",
            "angular_gain": "1.8",
            "base_linear_speed": "0.10",
            "stop_area_fraction": "1.0",
            "spawn_x": "2.3536",
            "spawn_y": "0.3536",
            "spawn_yaw": "1.9635",
        }.items(),
    )

    return LaunchDescription([sim])
