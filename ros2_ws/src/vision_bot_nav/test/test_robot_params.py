"""Tests for launch/robot_params.py (pure, no ROS needed).

Run: python3 -m pytest ros2_ws/src/vision_bot_nav/test
"""

import copy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "launch"))

from robot_params import robot_params  # noqa: E402

BASE = {
    "amcl": {"ros__parameters": {"scan_topic": "scan",
                                 "initial_pose": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}}},
    "local_costmap": {"local_costmap": {"ros__parameters": {
        "obstacle_layer": {"scan": {"topic": "/scan", "clearing": True}},
        "robot_avoidance_layer": {"robot_pose_topics": ["/robot2/shared_pose"]}}}},
    "global_costmap": {"global_costmap": {"ros__parameters": {
        "obstacle_layer": {"scan": {"topic": "/scan"}},
        "robot_avoidance_layer": {"robot_pose_topics": ["/robot2/shared_pose"]}}}},
    "bt_navigator": {"ros__parameters": {"odom_topic": "odometry/filtered"}},
}


def test_root_namespace_keeps_layout_and_sets_robot_values():
    out = robot_params(BASE, "", (1.0, 2.0, 0.5), ["/other/shared_pose"])
    assert set(out) == set(BASE)
    assert out["amcl"]["ros__parameters"]["initial_pose"] == {
        "x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}
    lc = out["local_costmap"]["local_costmap"]["ros__parameters"]
    assert lc["robot_avoidance_layer"]["robot_pose_topics"] == ["/other/shared_pose"]
    assert lc["obstacle_layer"]["scan"]["topic"] == "/scan"


def test_namespace_nests_everything_under_it():
    out = robot_params(BASE, "robot1", (0.0, 0.0, 0.0), ["/robot2/shared_pose"])
    assert list(out) == ["robot1"]
    assert out["robot1"]["bt_navigator"]["ros__parameters"]["odom_topic"] == "odometry/filtered"


def test_namespace_makes_costmap_scan_topics_absolute_per_robot():
    # A relative topic would resolve under the costmap's own namespace
    # (/robot1/local_costmap/scan); the installed multi-robot params use
    # /robot1/scan for the same reason.
    out = robot_params(BASE, "robot1", (0.0, 0.0, 0.0), [])["robot1"]
    for name in ("local_costmap", "global_costmap"):
        scan = out[name][name]["ros__parameters"]["obstacle_layer"]["scan"]
        assert scan["topic"] == "/robot1/scan"
    assert out["amcl"]["ros__parameters"]["scan_topic"] == "scan"   # node-relative, already right


def test_each_robot_watches_the_topics_it_is_given():
    one = robot_params(BASE, "robot1", (0.0, 0.0, 0.0), ["/robot2/shared_pose"])["robot1"]
    two = robot_params(BASE, "robot2", (-2.5, 0.0, 0.0), ["/robot1/shared_pose"])["robot2"]
    for name in ("local_costmap", "global_costmap"):
        assert one[name][name]["ros__parameters"]["robot_avoidance_layer"][
            "robot_pose_topics"] == ["/robot2/shared_pose"]
        assert two[name][name]["ros__parameters"]["robot_avoidance_layer"][
            "robot_pose_topics"] == ["/robot1/shared_pose"]
    assert two["amcl"]["ros__parameters"]["initial_pose"]["x"] == -2.5


def test_input_is_not_modified():
    before = copy.deepcopy(BASE)
    robot_params(BASE, "robot1", (3.0, 3.0, 1.0), ["/x"])
    assert BASE == before
