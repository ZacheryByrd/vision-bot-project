"""Tests for benchmarks/metrics.py (pure functions, no ROS needed)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metrics import ErrorStats, Pose2D, position_errors, relative_to  # noqa: E402


def test_identical_paths_zero_error():
    path = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 2.0, 0.5), Pose2D(3.0, -1.0, 1.0)]
    assert position_errors(path, path) == ErrorStats(mean=0.0, max=0.0, final=0.0)


def test_known_offset():
    truth = [Pose2D(0.0, 0.0, 0.0)] * 3
    estimate = [Pose2D(0.0, 0.0, 0.0), Pose2D(0.0, 0.0, 0.0), Pose2D(3.0, 4.0, 0.0)]
    stats = position_errors(estimate, truth)
    assert stats.mean == pytest.approx(5.0 / 3.0)
    assert stats.max == pytest.approx(5.0)
    assert stats.final == pytest.approx(5.0)


def test_length_mismatch_raises():
    with pytest.raises(ValueError):
        position_errors([Pose2D(0.0, 0.0, 0.0)], [Pose2D(0.0, 0.0, 0.0)] * 2)


def test_empty_paths_raise():
    with pytest.raises(ValueError):
        position_errors([], [])


def test_yaw_does_not_count_as_position_error():
    stats = position_errors([Pose2D(1.0, 1.0, 3.0)], [Pose2D(1.0, 1.0, -2.0)])
    assert stats.final == 0.0


def test_relative_to_own_origin_is_zero():
    origin = Pose2D(1.0, 2.0, 0.7)
    rel = relative_to(origin, origin)
    assert (rel.x, rel.y, rel.yaw) == pytest.approx((0.0, 0.0, 0.0))


def test_relative_to_rotates_into_origin_frame():
    # One metre "north" of an origin facing north is one metre straight ahead.
    rel = relative_to(Pose2D(1.0, 2.0, math.pi / 2), Pose2D(1.0, 3.0, math.pi / 2))
    assert (rel.x, rel.y, rel.yaw) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)


def test_relative_to_wraps_yaw():
    rel = relative_to(Pose2D(0.0, 0.0, 3.0), Pose2D(0.0, 0.0, -3.0))
    assert rel.yaw == pytest.approx(2 * math.pi - 6.0)


# --- Task 3: goal benchmark summary -------------------------------------

from metrics import GoalResult, GoalSummary, summarize_goals  # noqa: E402


def _goal(goal_id, success, time_s=10.0, error=0.1, recoveries=0):
    return GoalResult(goal_id=goal_id, success=success, time_s=time_s, path_length_m=1.0,
                      final_error_m=error, recoveries=recoveries)


def test_summary_counts_success_rate():
    results = [_goal("a", True), _goal("b", True), _goal("c", False), _goal("d", True)]
    summary = summarize_goals(results)
    assert summary.n == 4
    assert summary.success_rate == pytest.approx(0.75)


def test_means_use_successes_only():
    results = [_goal("a", True, time_s=10.0, error=0.1), _goal("b", True, time_s=20.0, error=0.3),
               _goal("c", False, time_s=999.0, error=9.0)]
    summary = summarize_goals(results)
    assert summary.mean_time_s == pytest.approx(15.0)
    assert summary.mean_final_error_m == pytest.approx(0.2)


def test_recoveries_count_every_goal():
    results = [_goal("a", True, recoveries=1), _goal("b", False, recoveries=3)]
    assert summarize_goals(results).total_recoveries == 4


def test_all_failed_gives_zero_means():
    summary = summarize_goals([_goal("a", False, time_s=50.0)])
    assert (summary.n, summary.success_rate, summary.mean_time_s) == (1, 0.0, 0.0)


def test_empty_results():
    assert summarize_goals([]) == GoalSummary(n=0, success_rate=0.0, mean_time_s=0.0,
                                              mean_final_error_m=0.0, total_recoveries=0)
