"""One active Nav2 goal per robot, driven by goals polled from the fleet (pure, no ROS).

A new goal replaces the active one: the old goal is cancelled in Nav2 and
reported "canceled" to the fleet at once, so the later Nav2 result for it is
stale and ignored. "accepted" is reported when Nav2 accepts the goal, not when
the bridge sends it; until then the goal is still pending in the fleet, so a
repeat of the active goal from the next poll is ignored.
"""

from typing import Callable, Optional

from fleet_bridge.client import Goal

# Nav2 outcome -> fleet goal status; all but "accepted" end the goal.
FLEET_STATUS = {"accepted": "accepted", "succeeded": "succeeded", "aborted": "failed",
                "rejected": "failed", "canceled": "canceled"}


class GoalManager:
    def __init__(self, send: Callable[[Goal], None], cancel: Callable[[str], None],
                 report: Callable[[str, str], None]):
        self._send = send
        self._cancel = cancel
        self._report = report
        self._active: Optional[str] = None
        self._goal: Optional[Goal] = None

    @property
    def active_goal_id(self) -> Optional[str]:
        return self._active

    @property
    def active_goal(self) -> Optional[Goal]:
        return self._goal if self._active is not None else None

    def on_new_goal(self, goal: Goal) -> None:
        if goal.goal_id == self._active:
            return
        if self._active is not None:
            self._cancel(self._active)
            self._report(self._active, "canceled")
        self._active = goal.goal_id
        self._goal = goal
        try:
            self._send(goal)
        except Exception:
            self._active = None
            self._report(goal.goal_id, "failed")

    def on_result(self, goal_id: str, status: str) -> None:
        """status is a Nav2 outcome: accepted, rejected, succeeded, aborted or canceled."""
        if status not in FLEET_STATUS:
            raise ValueError(f"unknown Nav2 outcome {status!r}")
        if goal_id != self._active:
            return
        self._report(goal_id, FLEET_STATUS[status])
        if status != "accepted":
            self._active = None


class NavDispatcher:
    """Orders goal sends and cancels for one Nav2 action server.

    Nav2's action server, on a cancel, terminates every goal it holds, including
    one that arrived after the cancel and is still waiting to start; that goal
    is aborted unseen (Gate 6). So while a goal is still in Nav2, a new goal is
    held back and sent when the old one finishes; a held goal that is replaced
    in turn is simply dropped (the fleet was already told it was canceled).
    """

    def __init__(self, send_now: Callable[[Goal], None], cancel_now: Callable[[str], None]):
        self._send_now = send_now
        self._cancel_now = cancel_now
        self._in_nav2 = set()
        self._held: Optional[Goal] = None

    def send(self, goal: Goal) -> None:
        if self._in_nav2:
            self._held = goal
            return
        self._in_nav2.add(goal.goal_id)
        self._send_now(goal)

    def cancel(self, goal_id: str) -> None:
        if self._held is not None and self._held.goal_id == goal_id:
            self._held = None
        elif goal_id in self._in_nav2:
            self._cancel_now(goal_id)

    def on_finished(self, goal_id: str) -> None:
        """Nav2 finished (or rejected) this goal."""
        self._in_nav2.discard(goal_id)
        if not self._in_nav2 and self._held is not None:
            goal, self._held = self._held, None
            self.send(goal)
