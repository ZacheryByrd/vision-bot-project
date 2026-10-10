"""Tests for the pure helpers in benchmarks/plugin_ab.py (no ROS needed)."""

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
