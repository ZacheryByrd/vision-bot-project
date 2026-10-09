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
