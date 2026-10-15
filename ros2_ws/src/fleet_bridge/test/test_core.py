"""Tests for fleet_bridge/core.py: the bridge's timing and recovery logic (no ROS).

Run: python3 -m pytest ros2_ws/src/fleet_bridge/test
"""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_bridge.client import FleetError, FleetUnavailable, Goal  # noqa: E402
from fleet_bridge.core import (  # noqa: E402
    BridgeCore,
    SimulatedBattery,
    backoff_s,
    yaw_from_quaternion,
)


class FakeClient:
    def __init__(self):
        self.down = False
        self.registered = []
        self.telemetry = []
        self.reports = []
        self.goals = []           # pending goals the fleet will hand out, oldest first
        self.fail_telemetry = None

    def _check(self):
        if self.down:
            raise FleetUnavailable("down")

    def register_robot(self, robot_id, name):
        self._check()
        self.registered.append((robot_id, name))
        return {"id": 7, "serial_number": robot_id}

    def post_telemetry(self, robot_id, pose, nav_state, battery_pct, battery_simulated=True):
        self._check()
        if self.fail_telemetry:
            raise self.fail_telemetry
        self.telemetry.append((pose, nav_state, round(battery_pct, 3)))

    def fetch_pending_goal(self, robot_id):
        self._check()
        return self.goals[0] if self.goals else None

    def report_goal_status(self, goal_id, status):
        self._check()
        self.reports.append((goal_id, status))
        if status != "canceled":
            self.goals = [g for g in self.goals if g.goal_id != goal_id]


class Nav2:
    def __init__(self):
        self.sent, self.cancelled = [], []

    def send(self, goal):
        self.sent.append(goal)

    def cancel(self, goal_id):
        self.cancelled.append(goal_id)


def core(client, nav=None):
    nav = nav or Nav2()
    return BridgeCore(client, "SIM1", "robot1", send=nav.send, cancel=nav.cancel,
                      telemetry_period_s=1.0, poll_period_s=1.5)


def test_backoff_doubles_up_to_a_cap():
    assert [backoff_s(n) for n in range(7)] == [1.0, 2.0, 4.0, 8.0, 16.0, 30.0, 30.0]


def test_registers_with_backoff_while_api_down_and_never_raises():
    client = FakeClient()
    client.down = True
    c = core(client)
    attempts = []
    for t in (0.0, 0.5, 1.0, 2.0, 3.0, 6.9, 7.0):
        if t == 7.0:
            client.down = False
        before = c.registration_attempts
        c.step(t)
        if c.registration_attempts > before:
            attempts.append(t)
    assert attempts == [0.0, 1.0, 3.0, 7.0]       # waits 1, 2, 4 s between tries
    assert c.registered and client.registered == [("SIM1", "robot1")]


def test_registration_refused_keeps_retrying():
    client = FakeClient()
    client.register_robot = lambda robot_id, name: (_ for _ in ()).throw(
        FleetError(403, "run the admin seed script"))
    c = core(client)
    c.step(0.0)
    c.step(1.0)
    assert not c.registered and c.registration_attempts == 2


def test_no_telemetry_before_the_first_pose():
    client = FakeClient()
    c = core(client)
    c.step(0.0)
    c.step(1.0)
    assert client.telemetry == []


def test_posts_telemetry_every_period_with_pose_state_and_battery():
    client = FakeClient()
    c = core(client)
    c.update_pose(1.0, 2.0, 0.5)
    for t in (0.0, 0.4, 1.0, 1.9, 2.0):
        c.step(t)
    assert client.telemetry == [((1.0, 2.0, 0.5), "idle", 100.0)] * 3


def test_nav_state_is_navigating_while_a_goal_is_active():
    client = FakeClient()
    c = core(client)
    c.update_pose(0.0, 0.0, 0.0)
    client.goals = [Goal("4", 1.0, 1.0, 0.0)]
    c.step(0.0)
    assert client.telemetry[-1][1] == "navigating"


def test_polls_goals_and_sends_them_to_nav2():
    client, nav = FakeClient(), Nav2()
    c = core(client, nav)
    c.step(0.0)
    client.goals = [Goal("4", 1.0, 1.0, 0.0)]
    c.step(1.0)
    assert nav.sent == []                 # next poll is 1.5 s after the first
    c.step(1.5)
    assert nav.sent == [Goal("4", 1.0, 1.0, 0.0)]


def test_nav2_results_reach_the_fleet():
    client, nav = FakeClient(), Nav2()
    c = core(client, nav)
    client.goals = [Goal("4", 1.0, 1.0, 0.0)]
    c.step(0.0)
    c.on_nav_result("4", "accepted")
    c.on_nav_result("4", "succeeded")
    c.step(0.1)
    assert client.reports == [("4", "accepted"), ("4", "succeeded")]


def test_reports_survive_an_api_outage_in_order():
    client, nav = FakeClient(), Nav2()
    c = core(client, nav)
    client.goals = [Goal("4", 1.0, 1.0, 0.0)]
    c.step(0.0)
    client.down = True
    c.on_nav_result("4", "accepted")
    c.on_nav_result("4", "aborted")
    c.step(1.0)
    c.step(2.0)
    assert client.reports == []
    client.down = False
    c.step(3.0)
    assert client.reports == [("4", "accepted"), ("4", "failed")]


def test_a_refused_report_is_dropped_not_retried_forever():
    client = FakeClient()
    c = core(client)
    client.goals = [Goal("4", 1.0, 1.0, 0.0)]
    c.step(0.0)
    client.report_goal_status = lambda goal_id, status: (_ for _ in ()).throw(
        FleetError(404, "goal not found"))
    c.on_nav_result("4", "succeeded")
    c.step(0.1)
    c.step(0.2)
    assert c.pending_reports == 0


def test_telemetry_errors_do_not_stop_goal_polling():
    client, nav = FakeClient(), Nav2()
    c = core(client, nav)
    c.update_pose(0.0, 0.0, 0.0)
    client.fail_telemetry = FleetError(500, "boom")
    client.goals = [Goal("4", 1.0, 1.0, 0.0)]
    c.step(0.0)
    assert nav.sent == [Goal("4", 1.0, 1.0, 0.0)]


def test_battery_drains_with_distance_and_stops_at_zero():
    b = SimulatedBattery(drain_pct_per_m=0.5)
    b.update(0.0, 0.0)
    b.update(3.0, 4.0)                # 5 m
    assert b.percent == pytest.approx(97.5)
    b.update(3.0, 4.0)
    assert b.percent == pytest.approx(97.5)
    b.update(3.0, 1000.0)
    assert b.percent == 0.0


def test_battery_reaches_telemetry():
    client = FakeClient()
    c = core(client)
    c.update_pose(0.0, 0.0, 0.0)
    c.update_pose(10.0, 0.0, 0.0)
    c.step(0.0)
    assert client.telemetry[-1][2] == pytest.approx(100.0 - 10.0 * c.battery.drain_pct_per_m)


@pytest.mark.parametrize("yaw", [0.0, 0.7, -2.5, math.pi - 0.01])
def test_yaw_from_quaternion(yaw):
    assert yaw_from_quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == \
        pytest.approx(yaw)
