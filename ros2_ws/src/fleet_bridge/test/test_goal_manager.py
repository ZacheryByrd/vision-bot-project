"""Tests for fleet_bridge/goal_manager.py (pure, no ROS needed).

Run: python3 -m pytest ros2_ws/src/fleet_bridge/test
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_bridge.client import Goal  # noqa: E402
from fleet_bridge.goal_manager import GoalManager  # noqa: E402

G1 = Goal("1", 1.0, 2.0, 0.0)
G2 = Goal("2", -1.0, 0.5, 1.57)


class Recorder:
    def __init__(self, fail_send=False):
        self.sent, self.cancelled, self.reports = [], [], []
        self.fail_send = fail_send

    def send(self, goal):
        if self.fail_send:
            raise RuntimeError("navigate_to_pose action server not available")
        self.sent.append(goal)

    def cancel(self, goal_id):
        self.cancelled.append(goal_id)

    def report(self, goal_id, status):
        self.reports.append((goal_id, status))


def manager(rec):
    return GoalManager(send=rec.send, cancel=rec.cancel, report=rec.report)


def test_new_goal_sends_and_sets_active():
    rec = Recorder()
    gm = manager(rec)
    assert gm.active_goal_id is None
    gm.on_new_goal(G1)
    assert rec.sent == [G1] and gm.active_goal_id == "1"
    assert rec.reports == []        # "accepted" waits for Nav2 to accept


def test_second_goal_cancels_first():
    rec = Recorder()
    gm = manager(rec)
    gm.on_new_goal(G1)
    gm.on_new_goal(G2)
    assert rec.cancelled == ["1"]
    assert rec.sent == [G1, G2]
    assert gm.active_goal_id == "2"
    assert ("1", "canceled") in rec.reports


def test_rejected_goal_reports_failed():
    rec = Recorder()
    gm = manager(rec)
    gm.on_new_goal(G1)
    gm.on_result("1", "rejected")
    assert rec.reports == [("1", "failed")]
    assert gm.active_goal_id is None


def test_result_for_stale_goal_ignored():
    rec = Recorder()
    gm = manager(rec)
    gm.on_new_goal(G1)
    gm.on_new_goal(G2)
    reports = list(rec.reports)
    gm.on_result("1", "canceled")      # Nav2 confirms the cancel of the replaced goal later
    gm.on_result("1", "succeeded")
    assert rec.reports == reports
    assert gm.active_goal_id == "2"


def test_same_goal_polled_again_is_not_resent():
    rec = Recorder()
    gm = manager(rec)
    gm.on_new_goal(G1)
    gm.on_new_goal(G1)              # still pending in the fleet until "accepted" lands
    assert rec.sent == [G1] and rec.cancelled == []


def test_accepted_is_reported_and_goal_stays_active():
    rec = Recorder()
    gm = manager(rec)
    gm.on_new_goal(G1)
    gm.on_result("1", "accepted")
    assert rec.reports == [("1", "accepted")]
    assert gm.active_goal_id == "1"


@pytest.mark.parametrize("nav2, fleet", [("succeeded", "succeeded"), ("aborted", "failed"),
                                         ("canceled", "canceled"), ("rejected", "failed")])
def test_final_results_map_to_fleet_statuses_and_clear_the_goal(nav2, fleet):
    rec = Recorder()
    gm = manager(rec)
    gm.on_new_goal(G1)
    gm.on_result("1", nav2)
    assert rec.reports == [("1", fleet)]
    assert gm.active_goal_id is None


def test_unknown_result_is_an_error():
    gm = manager(Recorder())
    gm.on_new_goal(G1)
    with pytest.raises(ValueError):
        gm.on_result("1", "exploded")


def test_send_failure_reports_failed():
    rec = Recorder(fail_send=True)
    gm = manager(rec)
    gm.on_new_goal(G1)
    assert rec.reports == [("1", "failed")]
    assert gm.active_goal_id is None


# NavDispatcher: Nav2 aborts a goal that arrives while it is still cancelling
# the previous one (found at Gate 6), so a replacement waits for the old goal
# to finish in Nav2.

class FakeNav2:
    def __init__(self):
        self.calls = []

    def send_now(self, goal):
        self.calls.append(("send", goal.goal_id))

    def cancel_now(self, goal_id):
        self.calls.append(("cancel", goal_id))


def dispatcher():
    from fleet_bridge.goal_manager import NavDispatcher
    nav = FakeNav2()
    return NavDispatcher(send_now=nav.send_now, cancel_now=nav.cancel_now), nav


def test_dispatcher_sends_straight_away_when_nav2_is_idle():
    d, nav = dispatcher()
    d.send(G1)
    assert nav.calls == [("send", "1")]


def test_replacement_waits_until_the_cancelled_goal_finishes():
    d, nav = dispatcher()
    d.send(G1)
    d.cancel("1")
    d.send(G2)
    assert nav.calls == [("send", "1"), ("cancel", "1")]
    d.on_finished("1")
    assert nav.calls[-1] == ("send", "2")


def test_a_deferred_goal_that_is_replaced_again_is_never_sent():
    G3 = Goal("3", 0.0, 0.0, 0.0)
    d, nav = dispatcher()
    d.send(G1)
    d.cancel("1")
    d.send(G2)
    d.cancel("2")
    d.send(G3)
    d.on_finished("1")
    assert nav.calls == [("send", "1"), ("cancel", "1"), ("send", "3")]


def test_finishing_an_unknown_goal_changes_nothing():
    d, nav = dispatcher()
    d.on_finished("9")
    d.send(G1)
    assert nav.calls == [("send", "1")]
