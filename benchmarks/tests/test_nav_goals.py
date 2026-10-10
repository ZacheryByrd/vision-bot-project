"""Tests for the pure helpers in benchmarks/nav_goals.py (no ROS needed)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav_goals import Goal, load_goals, path_length  # noqa: E402

GOALS_FILE = Path(__file__).resolve().parents[1] / "goals.yaml"


def test_load_goals_reads_ids_and_poses():
    goals = load_goals("goals:\n"
                       "  - {id: a, x: 1.0, y: -2.0, yaw: 0.5}\n"
                       "  - {id: b, x: 0, y: 0, yaw: 0}\n")
    assert goals == [Goal("a", 1.0, -2.0, 0.5), Goal("b", 0.0, 0.0, 0.0)]


def test_duplicate_ids_rejected():
    with pytest.raises(ValueError):
        load_goals("goals:\n  - {id: a, x: 1, y: 1, yaw: 0}\n  - {id: a, x: 2, y: 2, yaw: 0}\n")


def test_empty_goal_list_rejected():
    with pytest.raises(ValueError):
        load_goals("goals: []\n")


def test_benchmark_file_has_at_least_ten_goals():
    assert len(load_goals(GOALS_FILE.read_text())) >= 10


def test_path_length_sums_segments():
    assert path_length([(0.0, 0.0), (3.0, 4.0), (3.0, 0.0)]) == pytest.approx(9.0)


def test_path_length_of_one_point_is_zero():
    assert path_length([(1.0, 1.0)]) == 0.0
