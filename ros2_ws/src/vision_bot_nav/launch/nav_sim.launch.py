"""
nav_sim.launch.py
=================
v2 simulation bring-up: Gazebo Classic with the navigation world, the
vision_bot_nav robot model (v1 rover + lidar + IMU), robot_state_publisher,
the EKF (owner of odom -> base_link), and twist_mux as the single owner of
the drive command. slam.launch.py and the Nav2 launch build on this file.

Run with:
    ros2 launch vision_bot_nav nav_sim.launch.py              # GUI (VcXsrv)
    xvfb-run -a ros2 launch vision_bot_nav nav_sim.launch.py gui:=false

Velocity inputs (priorities in config/twist_mux.yaml):
    cmd_vel_teleop (100) > cmd_vel_line (20) > cmd_vel_nav (10)
        > cmd_vel_idle (1, always zero, from idle_stop.py)  ->  cmd_vel

Keyboard teleop needs its own terminal (it reads keys from a TTY), so it is
not started here. In a second shell in the container:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r cmd_vel:=cmd_vel_teleop

v1 line following through the mux: perception_node and motor_control_node
start only with line_follow:=true. When motor_control_node has no target it
keeps publishing a search spin, which would hold cmd_vel_line (priority 20)
and block navigation (priority 10) forever (docs/DECISIONS.md #4). Its
/cmd_vel output is remapped to cmd_vel_line here; v1 code is unchanged.
    ros2 launch vision_bot_nav nav_sim.launch.py line_follow:=true \
        world:=$(ros2 pkg prefix vision_bot)/share/vision_bot/worlds/line_track.world \
        spawn_x:=2.3536 spawn_y:=0.3536 spawn_yaw:=1.9635

`namespace` pushes the robot's ROS nodes and Gazebo plugins into a
namespace. TF is not namespaced yet; two-robot TF trees come in Task 5.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot_nav")
    xacro_file = os.path.join(pkg_share, "urdf", "vision_bot_nav.urdf.xacro")
    robot_description = xacro.process_file(xacro_file).toxml()

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time = ParameterValue(use_sim_time, value_type=bool)

    args = [
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="ROS namespace for this robot's nodes and topics."),
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(pkg_share, "worlds", "nav_world.world"),
            description="Full path to a Gazebo Classic world file."),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use Gazebo's /clock. Keep true in simulation."),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start the Gazebo client window (false for headless)."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Start RViz with rviz/nav.rviz."),
        DeclareLaunchArgument(
            "line_follow", default_value="false",
            description="Start v1 perception + motor control, feeding cmd_vel_line."),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        # Slightly above the floor so the wheels settle onto it instead of
        # starting inside it.
        DeclareLaunchArgument("spawn_z", default_value="0.04"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
            )
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": sim_time}],
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "vision_bot",
            "-robot_namespace", namespace,
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
            # Default is 30 s. With the full Nav2 stack starting alongside it,
            # gzserver took longer than that to offer /spawn_entity.
            "-timeout", "120",
        ],
        output="screen",
    )

    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "twist_mux.yaml"),
            {"use_sim_time": sim_time},
        ],
        remappings=[("cmd_vel_out", "cmd_vel")],
    )

    # EKF: fuses wheel odometry and the IMU gyro; the only publisher of
    # odom -> base_link, and of odometry/filtered.
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            os.path.join(pkg_share, "config", "ekf.yaml"),
            {"use_sim_time": sim_time},
        ],
    )

    # Lowest-priority zero input, so the robot stops when every real source
    # has timed out (see scripts/idle_stop.py).
    idle_stop = Node(
        package="vision_bot_nav",
        executable="idle_stop.py",
        output="screen",
        parameters=[{"use_sim_time": sim_time}],
    )

    # v1 nodes with the line-following tuning from vision_bot's
    # line_follow_launch.py. Parameters only; no v1 code changes.
    line_follow = IfCondition(LaunchConfiguration("line_follow"))
    perception = Node(
        package="vision_bot",
        executable="perception_node",
        name="perception_node",
        output="screen",
        condition=line_follow,
        parameters=[{
            "use_sim_time": sim_time,
            "image_topic": "/camera/image_raw",
            "target_hue_low": 25,
            "target_hue_high": 35,
            "target_sat_low": 120,
            "target_val_low": 80,
            "min_contour_area": 150,
        }],
    )
    motor_control = Node(
        package="vision_bot",
        executable="motor_control_node",
        name="motor_control_node",
        output="screen",
        condition=line_follow,
        parameters=[{
            "use_sim_time": sim_time,
            "angular_gain": 1.8,
            "base_linear_speed": 0.10,
            "stop_area_fraction": 1.0,
        }],
        remappings=[("/cmd_vel", "cmd_vel_line")],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz", "nav.rviz")],
        parameters=[{"use_sim_time": sim_time}],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    robot = GroupAction([
        PushRosNamespace(namespace),
        robot_state_publisher,
        spawn_entity,
        ekf,
        twist_mux,
        idle_stop,
        perception,
        motor_control,
        rviz,
    ])

    return LaunchDescription(args + [gazebo, robot])
