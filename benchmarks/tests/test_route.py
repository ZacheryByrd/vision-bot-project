"""Tests for the steering law and route definitions in benchmarks/route.py."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from metrics import Pose2D  # noqa: E402
from route import K_ANG, MAX_ANG, MAX_LIN, ROUTES, reached, route_length, steer  # noqa: E402

ORIGIN = Pose2D(0.0, 0.0, 0.0)


def test_goal_far_ahead_drives_straight_at_full_speed():
    lin, ang = steer(ORIGIN, (5.0, 0.0))
    assert lin == pytest.approx(MAX_LIN)
    assert ang == pytest.approx(0.0)


def test_goal_behind_turns_in_place():
    lin, ang = steer(ORIGIN, (-5.0, 0.1))
    assert lin == 0.0
    assert abs(ang) == pytest.approx(MAX_ANG)


def test_goal_to_the_left_turns_left_in_place():
    lin, ang = steer(ORIGIN, (0.0, 3.0))
    assert lin == 0.0
    assert ang == pytest.approx(MAX_ANG)


def test_goal_to_the_right_turns_right_in_place():
    lin, ang = steer(ORIGIN, (0.0, -3.0))
    assert lin == 0.0
    assert ang == pytest.approx(-MAX_ANG)


def test_small_heading_error_drives_while_correcting():
    goal = (5.0 * math.cos(0.2), 5.0 * math.sin(0.2))
    lin, ang = steer(ORIGIN, goal)
    assert lin > 0.0
    assert ang == pytest.approx(K_ANG * 0.2)


def test_slows_down_near_the_goal():
    lin, _ = steer(ORIGIN, (0.2, 0.0))
    assert 0.0 < lin < MAX_LIN


def test_heading_uses_robot_yaw():
    facing_left = Pose2D(0.0, 0.0, math.pi / 2)
    lin, ang = steer(facing_left, (0.0, 5.0))
    assert lin == pytest.approx(MAX_LIN)
    assert ang == pytest.approx(0.0, abs=1e-9)


def test_reached_within_tolerance_only():
    assert reached(ORIGIN, (0.05, 0.05))
    assert not reached(ORIGIN, (0.5, 0.0))


def test_route_length_from_start():
    assert route_length([(3.0, 4.0), (3.0, 0.0)], start=(0.0, 0.0)) == pytest.approx(9.0)


def test_drift_route_is_at_least_20_m():
    assert route_length(ROUTES["drift_loop"], start=(0.0, 0.0)) >= 20.0


def test_every_route_has_waypoints():
    assert all(len(waypoints) > 0 for waypoints in ROUTES.values())
