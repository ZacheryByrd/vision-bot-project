"""
sim_launch.py
=============
Brings up: Gazebo, the vision_bot robot model (spawned from xacro via
robot_state_publisher), and both ROS2 nodes (perception + motor control).

Run with:
    ros2 launch vision_bot sim_launch.py

Then drive manually first to sanity-check topics before trusting the
vision pipeline:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot")
    xacro_file = os.path.join(pkg_share, "description", "vision_bot.urdf.xacro")
    world_file = os.path.join(pkg_share, "worlds", "track.world")

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_file}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "vision_bot"],
        output="screen",
    )

    perception = Node(
        package="vision_bot",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{"image_topic": "/camera/image_raw"}],
    )

    motor_control = Node(
        package="vision_bot",
        executable="motor_control_node",
        name="motor_control_node",
        output="screen",
    )

    return LaunchDescription(
        [gazebo, robot_state_publisher, spawn_entity, perception, motor_control]
    )
