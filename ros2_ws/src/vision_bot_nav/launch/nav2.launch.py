"""
nav2.launch.py
==============
Autonomous navigation on the saved map for one robot: everything in
nav_sim.launch.py (sim, robot, EKF, twist_mux) plus map_server + AMCL
(localization; AMCL owns map -> odom), the Nav2 navigation servers, and a
shared_pose publisher for the other robots. The node set, lifecycle lists,
composition, namespacing, and /tf remaps follow the installed nav2_bringup
bringup_launch.py, localization_launch.py, and navigation_launch.py (Humble).

    xvfb-run -a ros2 launch vision_bot_nav nav2.launch.py gui:=false
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: 2.3, y: -0.3}, orientation: {w: 1.0}}}"
    (or click "2D Goal Pose" in RViz with rviz:=true, which publishes the same message)

Two robots: multi_robot.launch.py includes this file once per robot with
namespace, entity, spawn pose (also AMCL's initial pose), and the other
robot's shared_pose topic. Per-robot Nav2 parameters are generated from
config/nav2_params.yaml by robot_params.py (nested under the namespace,
costmap scan topics made absolute) and written to a temporary file.

Composition (use_composition:=true, the default, as in the stock bringup):
every Nav2 server runs as a component in one process, <ns>/nav2_container.
With one process per server, the navigation lifecycle manager sometimes
waited forever on a lifecycle service call; in one process the servers
discover each other immediately, and it saves CPU.

Velocity routing differs from the stock bringup on purpose. Stock Humble
sends the velocity smoother's output straight to cmd_vel and leaves the
behavior server (spin, back up) publishing on cmd_vel as well; both would
bypass twist_mux. Here every Nav2 velocity enters twist_mux on cmd_vel_nav:
    controller_server   cmd_vel                 -> cmd_vel_nav_unsmoothed
    velocity_smoother   cmd_vel_nav_unsmoothed  -> cmd_vel_nav
    behavior_server     cmd_vel                 -> cmd_vel_nav
"""

import os
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from robot_params import robot_params  # noqa: E402

LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = ["controller_server", "smoother_server", "planner_server",
                    "behavior_server", "bt_navigator", "waypoint_follower",
                    "velocity_smoother"]
TF_REMAPS = [("/tf", "tf"), ("/tf_static", "tf_static")]


def _nav2_actions(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    namespace = arg("namespace").strip("/")
    pose = (float(arg("spawn_x")), float(arg("spawn_y")), float(arg("spawn_yaw")))
    others = [t.strip() for t in arg("robot_pose_topics").split(",") if t.strip()]
    with open(arg("nav2_params")) as f:
        base = yaml.safe_load(f)
    fd, params = tempfile.mkstemp(prefix=f"nav2_params_{namespace or 'root'}_", suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(robot_params(base, namespace, pose, others), f)

    def lifecycle(node_names):
        return [{"use_sim_time": True, "autostart": True, "node_names": node_names}]

    # (package, executable, component plugin, node name, parameters, extra remappings)
    servers = [
        ("nav2_map_server", "map_server", "nav2_map_server::MapServer", "map_server",
         [params, {"yaml_filename": arg("map")}], []),
        ("nav2_amcl", "amcl", "nav2_amcl::AmclNode", "amcl", [params], []),
        ("nav2_lifecycle_manager", "lifecycle_manager",
         "nav2_lifecycle_manager::LifecycleManager", "lifecycle_manager_localization",
         lifecycle(LOCALIZATION_NODES), []),
        ("nav2_controller", "controller_server", "nav2_controller::ControllerServer",
         "controller_server", [params], [("cmd_vel", "cmd_vel_nav_unsmoothed")]),
        ("nav2_smoother", "smoother_server", "nav2_smoother::SmootherServer",
         "smoother_server", [params], []),
        ("nav2_planner", "planner_server", "nav2_planner::PlannerServer",
         "planner_server", [params], []),
        ("nav2_behaviors", "behavior_server", "behavior_server::BehaviorServer",
         "behavior_server", [params], [("cmd_vel", "cmd_vel_nav")]),
        ("nav2_bt_navigator", "bt_navigator", "nav2_bt_navigator::BtNavigator",
         "bt_navigator", [params], []),
        ("nav2_waypoint_follower", "waypoint_follower",
         "nav2_waypoint_follower::WaypointFollower", "waypoint_follower", [params], []),
        ("nav2_velocity_smoother", "velocity_smoother",
         "nav2_velocity_smoother::VelocitySmoother", "velocity_smoother", [params],
         [("cmd_vel", "cmd_vel_nav_unsmoothed"), ("cmd_vel_smoothed", "cmd_vel_nav")]),
        ("nav2_lifecycle_manager", "lifecycle_manager",
         "nav2_lifecycle_manager::LifecycleManager", "lifecycle_manager_navigation",
         lifecycle(NAVIGATION_NODES), []),
    ]

    if arg("use_composition").lower() in ("true", "1"):
        container_name = f"/{namespace}/nav2_container" if namespace else "/nav2_container"
        nav2 = [
            Node(package="rclcpp_components", executable="component_container_isolated",
                 name="nav2_container", output="screen",
                 parameters=[params, {"autostart": True}], remappings=TF_REMAPS),
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(package=pkg, plugin=plugin, name=name,
                                   parameters=node_params, remappings=TF_REMAPS + remaps)
                    for pkg, _, plugin, name, node_params, remaps in servers]),
        ]
    else:
        nav2 = [Node(package=pkg, executable=exe, name=name, output="screen",
                     parameters=node_params, remappings=TF_REMAPS + remaps)
                for pkg, exe, _, name, node_params, remaps in servers]

    # This robot's map-frame pose at 10 Hz for the other robots'
    # RobotAvoidanceLayer (AMCL's amcl_pose stops while the robot is parked).
    shared_pose = Node(package="vision_bot_nav", executable="pose_publisher.py",
                       output="screen", parameters=[{"use_sim_time": True}],
                       remappings=TF_REMAPS)

    return [GroupAction([PushRosNamespace(namespace), *nav2, shared_pose])]


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot_nav")

    args = [
        DeclareLaunchArgument(
            "map", default_value=os.path.join(pkg_share, "maps", "nav_world.yaml"),
            description="map_server YAML of the map to localize and plan on."),
        # Not "params_file": gazebo.launch.py has an argument of that name, and
        # launch arguments are shared with included files, so gzserver would
        # be started with the Nav2 parameters.
        DeclareLaunchArgument(
            "nav2_params", default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            description="Nav2 parameters file (shared by every robot)."),
        DeclareLaunchArgument(
            "use_composition", default_value="true",
            description="Run the Nav2 servers as components in one process."),
        DeclareLaunchArgument(
            "namespace", default_value="",
            description="Robot namespace, e.g. robot1 (empty: none)."),
        DeclareLaunchArgument(
            "robot_pose_topics", default_value="/robot2/shared_pose",
            description="Comma-separated pose topics of the other robots, "
                        "for RobotAvoidanceLayer."),
        DeclareLaunchArgument(
            "start_gazebo", default_value="true",
            description="Start Gazebo (false for every robot after the first)."),
        DeclareLaunchArgument("entity", default_value="vision_bot",
                              description="Gazebo model name; unique per robot."),
        DeclareLaunchArgument("with_camera", default_value="true",
                              description="Include the camera (v1 line following)."),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0",
                              description="Spawn pose, also AMCL's initial pose."),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start the Gazebo client window (false for headless)."),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Start RViz with rviz/nav.rviz in this robot's namespace."),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, "launch", "nav_sim.launch.py")),
        launch_arguments={
            name: LaunchConfiguration(name)
            for name in ("gui", "rviz", "namespace", "start_gazebo", "entity", "with_camera",
                         "spawn_x", "spawn_y", "spawn_yaw")
        }.items(),
    )

    return LaunchDescription(args + [sim, OpaqueFunction(function=_nav2_actions)])
