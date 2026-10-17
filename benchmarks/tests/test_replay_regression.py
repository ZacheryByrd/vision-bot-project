"""Tests for the pure scoring in benchmarks/replay_regression.py (no ROS needed)."""

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metrics import Pose2D  # noqa: E402
from replay_regression import align, passes, read_threshold, score  # noqa: E402


def test_align_pairs_each_estimate_with_the_nearest_truth_in_time():
    est = [(1.00, "e1"), (2.00, "e2"), (3.00, "e3")]
    truth = [(0.98, "t1"), (2.04, "t2"), (2.90, "t3")]
    assert align(est, truth, max_dt=0.05) == [("e1", "t1"), ("e2", "t2")]   # 3.00 vs 2.90: too far


def test_align_handles_empty_inputs():
    assert align([], [(1.0, "t")], max_dt=0.05) == []
    assert align([(1.0, "e")], [], max_dt=0.05) == []


def test_score_compares_paths_relative_to_their_own_start():
    # The estimate starts at the odom origin, the truth in the world frame
    # 2 m away and rotated; relative to their starts the paths match exactly
    # except for a 0.1 m offset on the last sample.
    truth = [Pose2D(2.0, 1.0, math.pi / 2), Pose2D(2.0, 2.0, math.pi / 2),
             Pose2D(2.0, 3.0, math.pi / 2)]
    est = [Pose2D(0.0, 0.0, 0.0), Pose2D(1.0, 0.0, 0.0), Pose2D(2.1, 0.0, 0.0)]
    stats = score(list(zip(est, truth)))
    assert stats.final == pytest.approx(0.1)
    assert stats.mean == pytest.approx(0.1 / 3)


def test_score_needs_samples():
    with pytest.raises(ValueError):
        score([])


def test_passes_at_or_below_the_threshold():
    assert passes(0.10, 0.10) and passes(0.09, 0.10)
    assert not passes(0.11, 0.10)


def test_read_threshold_from_the_baseline_file(tmp_path):
    f = tmp_path / "baseline.json"
    f.write_text(json.dumps({"threshold_m": 0.123, "baseline_mean_m": 0.08}))
    assert read_threshold(f) == pytest.approx(0.123)
