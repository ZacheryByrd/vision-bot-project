"""Tests for the pure helpers in benchmarks/perception_impl.py (no ROS needed)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception_impl import compare_detections, cpu_percent, summarize  # noqa: E402


def stat_line(utime, stime, comm="perception_node"):
    # /proc/<pid>/stat: pid (comm) state, then fields 4..; utime is field 14, stime 15.
    rest = ["S"] + ["0"] * 10 + [str(utime), str(stime)] + ["0"] * 37
    return f"4242 ({comm}) " + " ".join(rest)


def test_summarize_four_latencies():
    s = summarize([0.001, 0.002, 0.003, 0.004])
    assert s["n"] == 4
    assert s["mean_ms"] == pytest.approx(2.5)
    assert s["p50_ms"] == pytest.approx(2.5)
    assert s["max_ms"] == pytest.approx(4.0)


def test_summarize_p95_interpolates():
    # 21 values 0..20 ms: the 95th percentile sits exactly on 19 ms.
    s = summarize([i / 1000 for i in range(21)])
    assert s["p95_ms"] == pytest.approx(19.0)


def test_summarize_single_value():
    s = summarize([0.007])
    assert (s["n"], s["mean_ms"], s["p50_ms"], s["p95_ms"], s["max_ms"]) == \
        pytest.approx((1, 7.0, 7.0, 7.0, 7.0))


def test_summarize_empty_raises():
    with pytest.raises(ValueError):
        summarize([])


def test_cpu_percent_fifty_ticks_in_one_second():
    assert cpu_percent(stat_line(100, 20), stat_line(130, 40), 1.0, 100) == pytest.approx(50.0)


def test_cpu_percent_comm_with_spaces_and_parens():
    # comm is free text in parentheses; fields are counted after the last ')'.
    before = stat_line(0, 0, comm="perception (node) x")
    after = stat_line(200, 0, comm="perception (node) x")
    assert cpu_percent(before, after, 2.0, 100) == pytest.approx(100.0)


def test_compare_detections_identical():
    a = [[1.0, 0.1, -0.2, 0.05, 1.0], [0.0, 0.0, 0.0, 0.0, 0.0]]
    assert compare_detections(a, a) == {"frames": 2, "detected_differs": 0, "max_abs_diff": 0.0}


def test_compare_detections_counts_flips_and_max_difference():
    a = [[1.0, 0.1, -0.2, 0.05, 1.0], [1.0, 0.3, 0.0, 0.1, 0.9], [0.0, 0.0, 0.0, 0.0, 0.0]]
    b = [[1.0, 0.1, -0.2, 0.05, 1.0], [1.0, 0.3, 0.0, 0.1, 0.95], [1.0, 0.5, 0.5, 0.2, 1.0]]
    result = compare_detections(a, b)
    assert result["frames"] == 3
    assert result["detected_differs"] == 1
    # The flipped frame is counted, not folded into the float difference.
    assert result["max_abs_diff"] == pytest.approx(0.05)


def test_compare_detections_length_mismatch_raises():
    with pytest.raises(ValueError):
        compare_detections([[0.0] * 5], [])
