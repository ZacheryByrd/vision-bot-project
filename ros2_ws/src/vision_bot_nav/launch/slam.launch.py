"""
slam.launch.py
==============
Mapping bring-up: everything in nav_sim.launch.py (sim, robot, EKF,
twist_mux) plus slam_toolbox in online async mode. While it runs,
slam_toolbox owns map -> odom and the EKF owns odom -> base_link.

Map the world and save it (inside the container, from the repo root):
    xvfb-run -a ros2 launch vision_bot_nav slam.launch.py gui:=false
    python3 benchmarks/drive_route.py --route explore     # second shell
    ros2 run nav2_map_server map_saver_cli \
        -f ros2_ws/src/vision_bot_nav/maps/nav_world --ros-args -p use_sim_time:=true

drive_route.py publishes on cmd_vel_teleop, so it is "teleop by script";
driving by hand with teleop_twist_keyboard works the same way.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot_nav")

    args = [
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start the Gazebo client window (false for headless)."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Start RViz with rviz/nav.rviz."),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "nav_sim.launch.py")),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[os.path.join(pkg_share, "config", "slam_toolbox.yaml")],
    )

    return LaunchDescription(args + [sim, slam])
