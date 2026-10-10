"""
nav2.launch.py
==============
Autonomous navigation on the saved map: everything in nav_sim.launch.py
(sim, robot, EKF, twist_mux) plus map_server + AMCL (localization; AMCL
owns map -> odom) and the Nav2 navigation servers. The node set, lifecycle
lists, composition, and /tf remaps follow the installed nav2_bringup
bringup_launch.py, localization_launch.py, and navigation_launch.py (Humble).

    xvfb-run -a ros2 launch vision_bot_nav nav2.launch.py gui:=false
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
        "{header: {frame_id: map}, pose: {position: {x: 2.3, y: -0.3}, orientation: {w: 1.0}}}"
    (or click "2D Goal Pose" in RViz with rviz:=true, which publishes the same message)

Composition (use_composition:=true, the default, as in the stock bringup):
every Nav2 server runs as a component in one process, nav2_container. With
one process per server, the navigation lifecycle manager sometimes waited
forever on a lifecycle service call ("Configuring smoother_server" and no
reply); in one process the servers discover each other immediately. It also
saves CPU. use_composition:=false starts one process per server.

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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

LOCALIZATION_NODES = ["map_server", "amcl"]
NAVIGATION_NODES = ["controller_server", "smoother_server", "planner_server",
                    "behavior_server", "bt_navigator", "waypoint_follower",
                    "velocity_smoother"]


def generate_launch_description():
    pkg_share = get_package_share_directory("vision_bot_nav")
    params = LaunchConfiguration("nav2_params")
    composed = LaunchConfiguration("use_composition")
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
            "use_composition", default_value="true",
            description="Run the Nav2 servers as components in one process."),
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

    def lifecycle(node_names):
        return [{"use_sim_time": True, "autostart": True, "node_names": node_names}]

    # (package, executable, component plugin, node name, parameters, extra remappings)
    servers = [
        ("nav2_map_server", "map_server", "nav2_map_server::MapServer", "map_server",
         [params, {"yaml_filename": LaunchConfiguration("map")}], []),
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

    container = Node(
        package="rclcpp_components", executable="component_container_isolated",
        name="nav2_container", output="screen", condition=IfCondition(composed),
        parameters=[params, {"autostart": True}], remappings=tf_remaps)
    components = LoadComposableNodes(
        target_container="nav2_container", condition=IfCondition(composed),
        composable_node_descriptions=[
            ComposableNode(package=pkg, plugin=plugin, name=name, parameters=node_params,
                           remappings=tf_remaps + remaps)
            for pkg, _, plugin, name, node_params, remaps in servers])
    processes = [
        Node(package=pkg, executable=exe, name=name, output="screen",
             condition=UnlessCondition(composed), parameters=node_params,
             remappings=tf_remaps + remaps)
        for pkg, exe, _, name, node_params, remaps in servers]

    return LaunchDescription(args + [sim, container, components] + processes)
