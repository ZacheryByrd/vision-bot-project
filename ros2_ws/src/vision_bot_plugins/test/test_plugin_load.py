"""Launch test: a Nav2 costmap loads and activates RobotAvoidanceLayer without errors.

Runs Nav2's standalone costmap node with only this layer, a static
map -> base_link transform so the costmap can activate, and a lifecycle
manager that configures and activates it. Passes when the costmap logs that
the plugin is initialized and active, and nothing logs an error.
"""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
def generate_test_description():
    costmap = launch_ros.actions.Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        output="screen",
        parameters=[{
            "global_frame": "map",
            "robot_base_frame": "base_link",
            "rolling_window": True,
            "width": 3,
            "height": 3,
            "resolution": 0.05,
            "plugins": ["robot_avoidance_layer"],
            "robot_avoidance_layer.plugin": "vision_bot_plugins::RobotAvoidanceLayer",
            "robot_avoidance_layer.robot_pose_topics": ["/robot2/amcl_pose"],
        }],
    )
    static_tf = launch_ros.actions.Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "base_link"],
    )
    lifecycle = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_test",
        output="screen",
        # The standalone costmap executable creates /costmap/costmap and, unlike
        # the Nav2 servers, no bond, so bond checking is off for this test.
        parameters=[{"autostart": True, "node_names": ["costmap/costmap"],
                     "bond_timeout": 0.0}],
    )
    return (
        launch.LaunchDescription([
            costmap, static_tf, lifecycle, launch_testing.actions.ReadyToTest()]),
        {"costmap": costmap, "lifecycle": lifecycle},
    )


class TestRobotAvoidanceLayerLoads(unittest.TestCase):

    def test_layer_initialized(self, proc_output, costmap):
        proc_output.assertWaitFor(
            'Initialized plugin "robot_avoidance_layer"', process=costmap, timeout=60)
        proc_output.assertWaitFor(
            "RobotAvoidanceLayer 'robot_avoidance_layer': 1 pose topic(s)",
            process=costmap, timeout=10)

    def test_costmap_activated(self, proc_output, lifecycle):
        proc_output.assertWaitFor("Managed nodes are active", process=lifecycle, timeout=60)


@launch_testing.post_shutdown_test()
class TestNoErrors(unittest.TestCase):

    def test_no_errors_logged(self, proc_output, costmap):
        text = "".join(o.text.decode() for o in proc_output[costmap])
        self.assertNotIn("[ERROR]", text)
        self.assertNotIn("Failed to create", text)
