"""Tests for the drone's seeded random flight path (pure; no ROS, no Gazebo).

Run: python3 -m pytest ros2_ws/src/vision_bot_drone_sim/test
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vision_bot_drone_sim.flight_path import make_waypoints, pose_at  # noqa: E402

BOUNDS = (-5.0, 5.0, -4.0, 4.0)  # x_min, x_max, y_min, y_max


def test_same_seed_same_path():
    assert make_waypoints(7, BOUNDS) == make_waypoints(7, BOUNDS)


def test_different_seed_differs():
    assert make_waypoints(7, BOUNDS) != make_waypoints(8, BOUNDS)


def test_waypoints_within_limits():
    for w in make_waypoints(3, BOUNDS, n=500):
        assert BOUNDS[0] <= w.x <= BOUNDS[1] and BOUNDS[2] <= w.y <= BOUNDS[3]
        assert 2.0 <= w.z <= 6.0
        assert 0.5 <= w.speed <= 2.0


def test_pose_moves_at_leg_speed():
    wps = make_waypoints(11, BOUNDS)
    a, b = wps[0], wps[1]
    leg_time = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) / b.speed
    t = leg_time / 2
    (p1, v), (p2, _) = pose_at(wps, t), pose_at(wps, t + 0.1)
    assert math.dist(p1, p2) == pytest.approx(b.speed * 0.1, abs=1e-9)
    assert math.hypot(*v) == pytest.approx(b.speed, abs=1e-9)


def test_pose_wraps_after_last_leg():
    wps = make_waypoints(5, BOUNDS, n=4)
    legs = list(zip(wps, wps[1:] + wps[:1]))
    period = sum(math.dist((a.x, a.y, a.z), (b.x, b.y, b.z)) / b.speed for a, b in legs)
    p0, _ = pose_at(wps, 0.3)
    p1, _ = pose_at(wps, 0.3 + period)
    assert math.dist(p0, p1) < 1e-9


def test_map_bounds_of_nav_world():
    from vision_bot_drone_sim.drone_flight import map_bounds
    nav = Path(__file__).resolve().parents[2] / "vision_bot_nav" / "maps" / "nav_world.yaml"
    x_min, x_max, y_min, y_max = map_bounds(nav)
    # origin [-4.97, -5.01], 200 x 200 pixels at 0.05 m
    assert (x_min, y_min) == (-4.97, -5.01)
    assert x_max == pytest.approx(5.03) and y_max == pytest.approx(4.99)
