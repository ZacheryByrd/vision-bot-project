"""The bridge's timing and recovery logic, kept free of ROS so it can be unit tested.

``BridgeCore.step(now)`` runs on the bridge's fleet thread a few times a
second. In order it: registers the robot (retrying with exponential backoff
while the API is down or refuses, never giving up), delivers queued goal
status reports (kept in order through an outage), polls for a pending goal,
and posts telemetry. The ROS side feeds it poses (``update_pose``) and Nav2
outcomes (``on_nav_result``) from the executor thread.
"""

import logging
import math
import threading
from collections import deque

from fleet_bridge.client import FleetAuthError, FleetError, FleetUnavailable
from fleet_bridge.goal_manager import GoalManager

log = logging.getLogger(__name__)

# Nav2 can report success short of an unreachable goal: NavFn re-targets it to
# the nearest free cell within its tolerance and the controller checks arrival
# against the end of that path (Gate 7: 0.33 m from a goal on a wall). Success
# farther than this from the requested goal is reported as failed, the same
# "truly arrived" threshold the Gate 4 benchmark uses (DECISIONS #48).
ARRIVED_M = 0.30


def backoff_s(attempt, base_s=1.0, cap_s=30.0):
    """Wait before retry number attempt + 1: 1, 2, 4, ... seconds, capped."""
    return min(cap_s, base_s * 2 ** attempt)


def is_temporary(err):
    """Rate limiting (429) and server errors (5xx) pass; other refusals will not."""
    return err.status == 429 or (err.status is not None and err.status >= 500)


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class SimulatedBattery:
    """The sim has no battery: charge drops in proportion to distance driven."""

    def __init__(self, drain_pct_per_m=0.5):
        self.drain_pct_per_m = drain_pct_per_m
        self.distance_m = 0.0
        self._last = None

    def update(self, x, y):
        if self._last is not None:
            self.distance_m += math.dist(self._last, (x, y))
        self._last = (x, y)

    @property
    def percent(self):
        return max(0.0, 100.0 - self.drain_pct_per_m * self.distance_m)


class BridgeCore:
    def __init__(self, client, robot_id, robot_name, send, cancel, telemetry_period_s=1.0,
                 poll_period_s=1.5, battery=None):
        self._client = client
        self._robot_id = robot_id
        self._robot_name = robot_name
        self._telemetry_period = telemetry_period_s
        self._poll_period = poll_period_s
        self.battery = battery or SimulatedBattery()
        self._lock = threading.Lock()       # GoalManager and pose: fleet and ROS threads
        self._goals = GoalManager(send=send, cancel=cancel, report=self._queue_report)
        self._reports = deque()
        self._pose = None
        self.registered = False
        self.registration_attempts = 0
        self._next_register = 0.0
        self._next_telemetry = 0.0
        self._next_poll = 0.0

    @property
    def pending_reports(self):
        return len(self._reports)

    # -- called from the ROS executor thread -------------------------------
    def update_pose(self, x, y, yaw):
        with self._lock:
            self._pose = (x, y, yaw)
            self.battery.update(x, y)

    def on_nav_result(self, goal_id, status):
        with self._lock:
            goal = self._goals.active_goal
            if status == "succeeded" and goal is not None and goal.goal_id == goal_id \
                    and self._pose is not None:
                miss = math.dist(self._pose[:2], (goal.x, goal.y))
                if miss > ARRIVED_M:
                    log.warning("goal %s: Nav2 reported success %.2f m from the goal; "
                                "reporting failed", goal_id, miss)
                    status = "aborted"
            self._goals.on_result(goal_id, status)

    def _queue_report(self, goal_id, status):
        self._reports.append((goal_id, status))

    # -- called from the fleet thread --------------------------------------
    def step(self, now):
        if not self.registered:
            if now < self._next_register:
                return
            self._register(now)
            if not self.registered:
                return
        self._deliver_reports()
        if now >= self._next_poll:
            self._next_poll = now + self._poll_period
            self._poll()
        if now >= self._next_telemetry:
            self._next_telemetry = now + self._telemetry_period
            self._post_telemetry()

    def _register(self, now):
        self.registration_attempts += 1
        try:
            robot = self._client.register_robot(self._robot_id, self._robot_name)
        except (FleetUnavailable, FleetError) as err:
            wait = backoff_s(self.registration_attempts - 1)
            self._next_register = now + wait
            log.warning("registering %s failed (%s); retrying in %.0f s", self._robot_id, err, wait)
            return
        self.registered = True
        log.info("registered %s as fleet robot %s", self._robot_id, robot.get("id"))

    def _deliver_reports(self):
        while self._reports:
            goal_id, status = self._reports[0]
            try:
                self._client.report_goal_status(goal_id, status)
            except (FleetUnavailable, FleetAuthError) as err:
                log.warning("goal %s %s not delivered yet (%s)", goal_id, status, err)
                return
            except FleetError as err:
                if is_temporary(err):
                    log.warning("goal %s %s not delivered yet (%s)", goal_id, status, err)
                    return
                log.warning("fleet refused goal %s %s (%s); dropping it", goal_id, status, err)
            self._reports.popleft()

    def _poll(self):
        try:
            goal = self._client.fetch_pending_goal(self._robot_id)
        except (FleetUnavailable, FleetError) as err:
            log.warning("polling goals failed (%s)", err)
            return
        if goal is not None:
            with self._lock:
                self._goals.on_new_goal(goal)

    def _post_telemetry(self):
        with self._lock:
            pose = self._pose
            state = "navigating" if self._goals.active_goal_id is not None else "idle"
            battery = self.battery.percent
        if pose is None:
            return
        try:
            self._client.post_telemetry(self._robot_id, pose, state, battery)
        except (FleetUnavailable, FleetError) as err:
            log.warning("posting telemetry failed (%s)", err)
