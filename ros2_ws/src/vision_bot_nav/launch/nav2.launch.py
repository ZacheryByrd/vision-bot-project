"""
nav2.launch.py
==============
Autonomous navigation on the saved map: everything in nav_sim.launch.py
(sim, robot, EKF, twist_mux) plus map_server + AMCL (localization; AMCL
owns map -> odom) and the Nav2 navigation servers. The node set, lifecycle
lists, and /tf remaps follow the installed nav2_bringup
localization_launch.py and navigation_launch.py (Humble, non-composed).

    xvfb-run -a ros2 launch vision_bot_nav nav2.launch.py gui:=false
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: 2.3, y: -0.3}, orientation: {w: 1.0}}}"
    (or click "2D Goal Pose" in RViz with rviz:=true, which publishes the same message)

Velocity routing differs from the stock bringup on purpose. Stock Humble
sends the velocity smoother's output straight to cmd_vel and leaves the
behavior server (spin, back up) publishing on cmd_vel as well; both would
bypass twist_mux. Here every Nav2 velocity enters twist_mux on cmd_vel_nav:
    controller_server   cmd_vel                 -> cmd_vel_nav_unsmoothed
    velocity_smoother   cmd_vel_nav_unsmoothed  -> cmd_vel_nav
    behavior_server     cmd_vel                 -> cmd_vel_nav
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = ["controller_server", "smoother_server", "planner_server",
                    "behavior_server", "bt_navigator", "waypoint_follower",
                    "velocity_smoother"]


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot_nav")
    params = LaunchConfiguration("nav2_params")
    tf_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

    args = [
        DeclareLaunchArgument(
            "map", default_value=os.path.join(pkg_share, "maps", "nav_world.yaml"),
            description="map_server YAML of the map to localize and plan on."),
        # Not "params_file": gazebo.launch.py has an argument of that name, and
        # launch arguments are shared with included files, so gzserver would
        # be started with the Nav2 parameters.
        DeclareLaunchArgument(
            "nav2_params", default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            description="Nav2 parameters file."),
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

    def nav2_node(package, executable, remappings=(), extra_params=()):
        return Node(package=package, executable=executable, name=executable, output="screen",
                    parameters=[params, *extra_params], remappings=tf_remaps + list(remappings))

    localization = [
        nav2_node("nav2_map_server", "map_server",
                  extra_params=[{"yaml_filename": LaunchConfiguration("map")}]),
        nav2_node("nav2_amcl", "amcl"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": LOCALIZATION_NODES}]),
    ]

    navigation = [
        nav2_node("nav2_controller", "controller_server",
                  remappings=[("cmd_vel", "cmd_vel_nav_unsmoothed")]),
        nav2_node("nav2_smoother", "smoother_server"),
        nav2_node("nav2_planner", "planner_server"),
        nav2_node("nav2_behaviors", "behavior_server",
                  remappings=[("cmd_vel", "cmd_vel_nav")]),
        nav2_node("nav2_bt_navigator", "bt_navigator"),
        nav2_node("nav2_waypoint_follower", "waypoint_follower"),
        nav2_node("nav2_velocity_smoother", "velocity_smoother",
                  remappings=[("cmd_vel", "cmd_vel_nav_unsmoothed"),
                              ("cmd_vel_smoothed", "cmd_vel_nav")]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": True, "autostart": True,
                          "node_names": NAVIGATION_NODES}]),
    ]

    return LaunchDescription(args + [sim] + localization + navigation)
