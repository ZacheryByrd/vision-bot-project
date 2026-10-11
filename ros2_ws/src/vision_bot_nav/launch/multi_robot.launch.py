"""
multi_robot.launch.py
=====================
Two robots on the shared map, based on the installed nav2_bringup
unique_multi_tb3_simulation_launch.py: one Gazebo, then one complete stack
per robot (robot_state_publisher, EKF, twist_mux, AMCL, Nav2, shared_pose)
in its own namespace with its own TF tree (/robotN/tf), each loading the
same maps/nav_world.yaml. Each robot's RobotAvoidanceLayer watches the
other robot's shared_pose.

    xvfb-run -a ros2 launch vision_bot_nav multi_robot.launch.py gui:=false
    ros2 topic pub --once /robot1/goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: 2.3, y: -0.3}, orientation: {w: 1.0}}}"

The cameras are left out (with_camera:=false): v1 line following is not
used here, and two rendering cameras would cost the simulation real time.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Spawn poses are also AMCL's initial poses (the map frame is the world frame).
ROBOTS = [
    {"name": "robot1", "x": 0.0, "y": 0.0, "yaw": 0.0, "other": "robot2"},
    {"name": "robot2", "x": -2.5, "y": 0.0, "yaw": 0.0, "other": "robot1"},
]


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot_nav")

    args = [
        DeclareLaunchArgument(
            "world", default_value=os.path.join(pkg_share, "worlds", "nav_world.world"),
            description="Full path to a Gazebo Classic world file."),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start the Gazebo client window (false for headless)."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Start one RViz per robot, each in its namespace."),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch",
                         "gazebo.launch.py")),
        launch_arguments={"world": LaunchConfiguration("world"),
                          "gui": LaunchConfiguration("gui")}.items(),
    )

    robots = [
        # GroupAction scopes the launch arguments, so robot2's values do not
        # leak into robot1's include.
        GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "nav2.launch.py")),
            launch_arguments={
                "namespace": r["name"],
                "entity": r["name"],
                "start_gazebo": "false",
                "with_camera": "false",
                "spawn_x": str(r["x"]),
                "spawn_y": str(r["y"]),
                "spawn_yaw": str(r["yaw"]),
                "robot_pose_topics": f"/{r['other']}/shared_pose",
                "gui": "false",
                "rviz": LaunchConfiguration("rviz"),
            }.items())])
        for r in ROBOTS
    ]

    return LaunchDescription(args + [gazebo] + robots)
