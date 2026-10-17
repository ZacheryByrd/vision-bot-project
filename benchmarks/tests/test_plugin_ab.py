"""Tests for the pure helpers in benchmarks/plugin_ab.py (no ROS needed)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metrics import GoalResult  # noqa: E402
from plugin_ab import (  # noqa: E402
    CONTACT_M,
    SCENARIOS,
    TrialRow,
    make_trials,
    min_separation,
    summarize_ab,
)


def test_trials_are_identical_for_both_conditions():
    trials = make_trials(per_scenario=10, base_seed=0)
    off = [(t.scenario, t.seed, t.robot2_start, t.robot2_delay_s)
           for t in trials if not t.layer_enabled]
    on = [(t.scenario, t.seed, t.robot2_start, t.robot2_delay_s)
          for t in trials if t.layer_enabled]
    assert sorted(off) == sorted(on)


def test_trial_count_and_balance():
    trials = make_trials(per_scenario=10, base_seed=0)
    assert len(trials) == 10 * len(SCENARIOS) * 2
    for name in SCENARIOS:
        for enabled in (False, True):
            assert sum(1 for t in trials if t.scenario == name and t.layer_enabled == enabled) == 10


def test_trials_are_deterministic():
    assert make_trials(per_scenario=3, base_seed=7) == make_trials(per_scenario=3, base_seed=7)
    assert make_trials(per_scenario=3, base_seed=7) != make_trials(per_scenario=3, base_seed=8)


def test_condition_order_alternates_so_neither_always_runs_first():
    trials = make_trials(per_scenario=4, base_seed=0)
    firsts = [trials[i].layer_enabled for i in range(0, len(trials), 2)]
    assert True in firsts and False in firsts


def test_parked_robot_stays_near_the_doorway_centre():
    for t in make_trials(per_scenario=20, base_seed=0):
        if t.scenario == "parked_doorway":
            x, y, _ = t.robot2_start
            assert x == pytest.approx(1.5)
            assert -0.4 <= y <= -0.2


def test_min_separation_aligns_samples_in_time():
    a = [(0.0, 0.0, 0.0), (1.0, 1.0, 0.0), (2.0, 2.0, 0.0)]
    b = [(0.0, 5.0, 0.0), (1.02, 1.0, 0.3), (2.0, 2.0, 4.0)]
    assert min_separation(a, b) == pytest.approx(0.3)


def test_min_separation_ignores_samples_too_far_apart_in_time():
    a = [(0.0, 0.0, 0.0)]
    b = [(5.0, 0.0, 0.1)]
    assert min_separation(a, b) is None


def _row(scenario, enabled, success, sep, recoveries=0, time_s=10.0):
    result = GoalResult(goal_id=f"{scenario}", success=success, time_s=time_s,
                        path_length_m=1.0, final_error_m=0.1, recoveries=recoveries)
    return TrialRow(scenario=scenario, seed=0, layer_enabled=enabled, result=result,
                    min_separation_m=sep, status="succeeded" if success else "aborted")


def test_summarize_ab_groups_by_scenario_and_condition():
    rows = [_row("crossing", False, True, 0.5), _row("crossing", False, False, 0.1, 3),
            _row("crossing", True, True, 0.6), _row("crossing", True, True, 0.4)]
    summary = summarize_ab(rows)
    off, on = summary[("crossing", False)], summary[("crossing", True)]
    assert off.goals.n == 2 and off.goals.success_rate == pytest.approx(0.5)
    assert off.contacts == 1 and off.worst_separation_m == pytest.approx(0.1)
    assert on.contacts == 0 and on.mean_separation_m == pytest.approx(0.5)
    assert off.goals.total_recoveries == 3


def test_contact_threshold_is_strict():
    rows = [_row("crossing", True, True, CONTACT_M), _row("crossing", True, True, CONTACT_M - 0.01)]
    assert summarize_ab(rows)[("crossing", True)].contacts == 1


def test_arrival_needs_success_and_true_position():
    # Nav2 can report success after a collision shoved the robot and
    # corrupted its localization; ground truth decides whether it arrived.
    from plugin_ab import ARRIVED_M
    close = GoalResult("a", True, 10.0, 1.0, ARRIVED_M - 0.01, 0)
    far = GoalResult("b", True, 10.0, 1.0, 0.8, 0)
    failed = GoalResult("c", False, 10.0, 1.0, 0.05, 0)
    rows = [TrialRow("parked_doorway", 0, False, r, 0.5, "x") for r in (close, far, failed)]
    assert summarize_ab(rows)[("parked_doorway", False)].arrivals == 1


def test_robot_io_without_namespace_matches_nav2_launch():
    from plugin_ab import robot_io
    io = robot_io("")
    assert io.entity == "vision_bot"
    assert io.truth == "ground_truth/pose"
    assert io.initialpose == "initialpose"
    assert io.navigate == "navigate_to_pose"
    assert "global_costmap/clear_entirely_global_costmap" in io.clear
    assert "local_costmap/local_costmap/set_parameters" in io.params


def test_robot_io_with_namespace_uses_it_for_entity_topics_and_services():
    from plugin_ab import robot_io
    io = robot_io("robot2")
    assert io.entity == "robot2"
    assert io.truth == "/robot2/ground_truth/pose"
    assert io.initialpose == "/robot2/initialpose"
    assert io.navigate == "/robot2/navigate_to_pose"
    assert set(io.clear) == {"/robot2/global_costmap/clear_entirely_global_costmap",
                             "/robot2/local_costmap/clear_entirely_local_costmap"}
    assert set(io.params) == {"/robot2/global_costmap/global_costmap/set_parameters",
                              "/robot2/local_costmap/local_costmap/set_parameters"}


def test_real_robot2_crosses_the_route_only_in_the_crossing_scenario():
    assert SCENARIOS["parked_doorway"].robot2_goal is None
    goal = SCENARIOS["crossing"].robot2_goal
    start = make_trials(per_scenario=1, base_seed=0)[2].robot2_start   # a crossing trial
    route_y = SCENARIOS["crossing"].start[1]
    assert goal[0] == pytest.approx(start[0])           # straight south along x = 0.3
    assert goal[1] < route_y < start[1]                 # from north of the route to south of it
    assert goal[2] == pytest.approx(start[2])


def _stamped(with_covariance):
    """Duck-typed stand-ins for PoseStamped and PoseWithCovarianceStamped."""
    from types import SimpleNamespace as NS
    pose = NS(position=NS(x=0.0, y=0.0), orientation=NS(z=0.0, w=1.0))
    return NS(header=NS(frame_id="", stamp=None),
              pose=NS(pose=pose, covariance=[0.0] * 36) if with_covariance else pose)


@pytest.mark.parametrize("with_covariance", [False, True])
def test_set_map_pose_fills_goal_and_initial_pose_messages(with_covariance):
    from plugin_ab import set_map_pose
    msg = set_map_pose(_stamped(with_covariance), 1.5, -0.3, math.pi / 2, stamp="now")
    pose = msg.pose.pose if with_covariance else msg.pose
    assert msg.header.frame_id == "map" and msg.header.stamp == "now"
    assert (pose.position.x, pose.position.y) == (1.5, -0.3)
    assert pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert pose.orientation.w == pytest.approx(math.sqrt(0.5))


def test_robot2_arrivals_counts_its_own_goal_per_condition():
    from plugin_ab import robot2_arrivals
    r = GoalResult(goal_id="g", success=True, time_s=1.0, path_length_m=1.0,
                   final_error_m=0.1, recoveries=0)
    rows = [TrialRow("crossing", 0, True, r, 0.5, "succeeded", "succeeded"),
            TrialRow("crossing", 1, True, r, 0.5, "succeeded", "cancelled at trial end"),
            TrialRow("crossing", 0, False, r, 0.5, "succeeded", "succeeded"),
            TrialRow("parked_doorway", 0, True, r, 0.5, "succeeded", "no goal")]
    assert robot2_arrivals(rows, "crossing", True) == (1, 2)
    assert robot2_arrivals(rows, "crossing", False) == (1, 1)
    assert robot2_arrivals(rows, "parked_doorway", True) == (0, 1)
