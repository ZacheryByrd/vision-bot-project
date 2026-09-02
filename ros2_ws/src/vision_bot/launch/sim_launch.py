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

Defaults to the red-box object-tracking demo (worlds/track.world). The
perception/control tuning is exposed as launch arguments specifically so
line_follow_launch.py can reuse this same file for the line-following
demo (worlds/line_track.world) with different values instead of
duplicating the whole node graph -- same nodes, same topics, different
world + color + gains.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot")
    xacro_file = os.path.join(pkg_share, "description", "vision_bot.urdf.xacro")

    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch the Gazebo client GUI window. Set to false for "
        "headless runs (CI, or no display attached -- e.g. `xvfb-run ros2 "
        "launch vision_bot sim_launch.py gui:=false`).",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="track.world",
        description="World file name (looked up under worlds/), e.g. "
        "track.world (red-box demo) or line_track.world (line-following demo).",
    )
    # Perception tuning -- see perception_node.py for what each one does.
    # Defaults match the red-box demo's HSV threshold.
    hue_low_arg = DeclareLaunchArgument("target_hue_low", default_value="0")
    hue_high_arg = DeclareLaunchArgument("target_hue_high", default_value="10")
    sat_low_arg = DeclareLaunchArgument("target_sat_low", default_value="120")
    val_low_arg = DeclareLaunchArgument("target_val_low", default_value="80")
    min_area_arg = DeclareLaunchArgument("min_contour_area", default_value="200")
    # DNN backend -- see perception_node.py / dnn_follow_launch.py. Defaults
    # keep the HSV backend active; dnn_follow_launch.py overrides these.
    detector_backend_arg = DeclareLaunchArgument("detector_backend", default_value="hsv")
    dnn_prototxt_arg = DeclareLaunchArgument("dnn_prototxt_path", default_value="")
    dnn_model_arg = DeclareLaunchArgument("dnn_model_path", default_value="")
    dnn_class_id_arg = DeclareLaunchArgument("dnn_target_class_id", default_value="15")
    dnn_confidence_arg = DeclareLaunchArgument("dnn_confidence_threshold", default_value="0.5")
    # Control tuning -- see motor_control_node.py. Defaults match the
    # red-box demo (approach-and-stop). The line-following demo overrides
    # stop_area_fraction to effectively disable stopping (drive continuously)
    # and uses a punchier angular_gain for the octagon track's 45-degree turns.
    angular_gain_arg = DeclareLaunchArgument("angular_gain", default_value="1.0")
    base_speed_arg = DeclareLaunchArgument("base_linear_speed", default_value="0.15")
    stop_area_arg = DeclareLaunchArgument("stop_area_fraction", default_value="0.15")
    # Where the robot spawns -- track.world's target is right in front of the
    # origin, so the object-tracking demo needs no override. The line-follow
    # demo spawns the robot directly on its track, facing along the line.
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="0.0")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="0.0")
    spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.0")
    spawn_yaw_arg = DeclareLaunchArgument("spawn_yaw", default_value="0.0")

    world_file = PathJoinSubstitution(
        [pkg_share, "worlds", LaunchConfiguration("world")]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": world_file,
            "gui": LaunchConfiguration("gui"),
        }.items(),
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
        arguments=[
            "-topic", "robot_description",
            "-entity", "vision_bot",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
        output="screen",
    )

    def int_param(name):
        return ParameterValue(LaunchConfiguration(name), value_type=int)

    def float_param(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    perception = Node(
        package="vision_bot",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{
            "image_topic": "/camera/image_raw",
            "target_hue_low": int_param("target_hue_low"),
            "target_hue_high": int_param("target_hue_high"),
            "target_sat_low": int_param("target_sat_low"),
            "target_val_low": int_param("target_val_low"),
            "min_contour_area": int_param("min_contour_area"),
            "detector_backend": LaunchConfiguration("detector_backend"),
            "dnn_prototxt_path": LaunchConfiguration("dnn_prototxt_path"),
            "dnn_model_path": LaunchConfiguration("dnn_model_path"),
            "dnn_target_class_id": int_param("dnn_target_class_id"),
            "dnn_confidence_threshold": float_param("dnn_confidence_threshold"),
        }],
    )

    motor_control = Node(
        package="vision_bot",
        executable="motor_control_node",
        name="motor_control_node",
        output="screen",
        parameters=[{
            "angular_gain": float_param("angular_gain"),
            "base_linear_speed": float_param("base_linear_speed"),
            "stop_area_fraction": float_param("stop_area_fraction"),
        }],
    )

    return LaunchDescription([
        gui_arg,
        world_arg,
        hue_low_arg,
        hue_high_arg,
        sat_low_arg,
        val_low_arg,
        min_area_arg,
        detector_backend_arg,
        dnn_prototxt_arg,
        dnn_model_arg,
        dnn_class_id_arg,
        dnn_confidence_arg,
        angular_gain_arg,
        base_speed_arg,
        stop_area_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        perception,
        motor_control,
    ])
