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

    @property
    def active_goal_id(self) -> Optional[str]:
        return self._active

    def on_new_goal(self, goal: Goal) -> None:
        if goal.goal_id == self._active:
            return
        if self._active is not None:
            self._cancel(self._active)
            self._report(self._active, "canceled")
        self._active = goal.goal_id
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
