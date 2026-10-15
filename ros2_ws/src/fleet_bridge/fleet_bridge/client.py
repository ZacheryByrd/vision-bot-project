"""HTTP client for the robot-fleet-management API (pure Python, no ROS).

Auth: log in with an account's email and password, send the access token as a
Bearer header, and on HTTP 401 refresh once and retry; if the refresh is also
refused, log in again and retry once more. Refresh rotates the refresh token,
so the new one replaces the old. Tokens and the password are never logged.

Robots are identified by serial number (the bridge's ``robot_id``); the
fleet's integer id is resolved once by ``register_robot``. Endpoint paths and
fields are those in docs/V2_DISCOVERY.md plus the approved goal and telemetry
additions (DECISIONS #2).
"""

import json as jsonlib
import logging
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Optional, Protocol
from urllib.parse import urlencode

log = logging.getLogger(__name__)

GOAL_STATUSES = ("accepted", "succeeded", "failed", "canceled")
PAGE = 100


@dataclass(frozen=True)
class Goal:
    goal_id: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class HttpResponse:
    status_code: int
    body: bytes = b""

    def json(self):
        return jsonlib.loads(self.body) if self.body else None


class HttpSession(Protocol):
    def request(self, method: str, url: str, *, headers: dict, json: Optional[dict] = None,
                form: Optional[dict] = None, timeout: float) -> HttpResponse:
        """Send one request; raise OSError (ConnectionError, TimeoutError) if no answer."""


class UrllibSession:
    """HttpSession over the standard library, so the bridge needs no extra packages."""

    def request(self, method, url, *, headers, json=None, form=None, timeout):
        headers = dict(headers)
        data = None
        if json is not None:
            data = jsonlib.dumps(json).encode()
            headers["Content-Type"] = "application/json"
        elif form is not None:
            data = urlencode(form).encode()
            headers["Content-Type"] = "application/x-www-form-urlencoded"
        req = urllib.request.Request(url, data=data, headers=headers, method=method)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                return HttpResponse(resp.status, resp.read())
        except urllib.error.HTTPError as err:          # 4xx/5xx still carry an answer
            return HttpResponse(err.code, err.read())


class FleetUnavailable(Exception):
    """The fleet API did not answer (down, refusing connections, or timing out)."""


class FleetError(Exception):
    """The fleet API answered with an error status."""

    def __init__(self, status, detail):
        super().__init__(f"fleet API returned {status}: {detail}")
        self.status = status
        self.detail = detail


class FleetAuthError(FleetError):
    """Login failed, or a request was still unauthorized after refreshing and logging in."""


def _detail(resp):
    try:
        body = resp.json()
    except ValueError:
        return resp.body[:200].decode(errors="replace")
    return body.get("detail", body) if isinstance(body, dict) else body


class FleetClient:
    def __init__(self, base_url: str, username: str, password: str, http: HttpSession,
                 timeout_s: float = 5.0):
        self._base = base_url.rstrip("/")
        self._username = username
        self._password = password
        self._http = http
        self._timeout = timeout_s
        self._access = None
        self._refresh = None
        self._robot_ids = {}            # serial number -> fleet integer id

    def __repr__(self):
        return f"FleetClient({self._base!r}, user={self._username!r})"

    # -- transport ---------------------------------------------------------
    def _send(self, method, path, *, json=None, form=None, params=None, token=None):
        url = self._base + path + (f"?{urlencode(params)}" if params else "")
        headers = {"Accept": "application/json"}
        if token:
            headers["Authorization"] = f"Bearer {token}"
        try:
            return self._http.request(method, url, headers=headers, json=json, form=form,
                                      timeout=self._timeout)
        except OSError as err:          # ConnectionError, TimeoutError, URLError
            raise FleetUnavailable(f"{method} {path}: {type(err).__name__}") from None

    def _store(self, resp):
        body = resp.json()
        self._access, self._refresh = body["access_token"], body["refresh_token"]

    def login(self) -> None:
        resp = self._send("POST", "/auth/login",
                          form={"username": self._username, "password": self._password})
        if resp.status_code != 200:
            raise FleetAuthError(resp.status_code, _detail(resp))
        self._store(resp)
        log.info("logged in to the fleet API as %s", self._username)

    def _try_refresh(self):
        if not self._refresh:
            return False
        resp = self._send("POST", "/auth/refresh", json={"refresh_token": self._refresh})
        if resp.status_code != 200:
            log.info("token refresh refused (%s); logging in again", resp.status_code)
            return False
        self._store(resp)
        log.info("access token refreshed")
        return True

    def request(self, method, path, *, json=None, params=None, ok=(200, 201, 204)):
        """Authenticated call with the refresh-then-login retry; returns the JSON body."""
        if self._access is None:
            self.login()
        resp = self._send(method, path, json=json, params=params, token=self._access)
        if resp.status_code == 401:
            log.info("%s %s unauthorized; refreshing", method, path)
            if not self._try_refresh():
                self.login()
            resp = self._send(method, path, json=json, params=params, token=self._access)
            if resp.status_code == 401:
                raise FleetAuthError(401, _detail(resp))
        if resp.status_code not in ok:
            raise FleetError(resp.status_code, _detail(resp))
        return resp.json()

    def _fleet_id(self, robot_id):
        if robot_id not in self._robot_ids:
            raise FleetError(None, f"robot {robot_id!r} is not resolved; call register_robot first")
        return self._robot_ids[robot_id]

    # -- API ---------------------------------------------------------------
    def register_robot(self, robot_id: str, name: str) -> dict:
        """Find the robot whose serial number is robot_id, or create it (admin accounts only)."""
        skip = 0
        while True:
            page = self.request("GET", "/robots", params={"skip": skip, "limit": PAGE})
            for robot in page["items"]:
                if robot["serial_number"] == robot_id:
                    self._robot_ids[robot_id] = robot["id"]
                    log.info("robot %s is fleet robot %s", robot_id, robot["id"])
                    return robot
            skip += len(page["items"])
            if not page["items"] or skip >= page["total"]:
                break
        try:
            robot = self.request("POST", "/robots", json={
                "name": name, "serial_number": robot_id, "model": "vision_bot (simulated)"})
        except FleetError as err:
            if err.status == 403:
                raise FleetError(403, f"robot {robot_id!r} is not in the fleet and this account "
                                      "cannot create robots; run the admin seed script") from None
            raise
        self._robot_ids[robot_id] = robot["id"]
        log.info("created fleet robot %s for %s", robot["id"], robot_id)
        return robot

    def post_telemetry(self, robot_id: str, pose: tuple, nav_state: str, battery_pct: float,
                       battery_simulated: bool = True) -> None:
        x, y, yaw = pose
        self.request("POST", f"/telemetry/robots/{self._fleet_id(robot_id)}", json={
            "pose_x": x, "pose_y": y, "pose_yaw": yaw, "nav_state": nav_state,
            "battery_level": battery_pct, "battery_simulated": battery_simulated})

    def fetch_pending_goal(self, robot_id: str) -> Optional[Goal]:
        """The oldest pending goal for this robot, or None."""
        goals = self.request("GET", f"/robots/{self._fleet_id(robot_id)}/goals",
                             params={"status": "pending"})
        if not goals:
            return None
        g = min(goals, key=lambda goal: goal["id"])
        return Goal(str(g["id"]), float(g["x"]), float(g["y"]), float(g["yaw"]))

    def report_goal_status(self, goal_id: str, status: str) -> None:
        if status not in GOAL_STATUSES:
            raise ValueError(f"status must be one of {GOAL_STATUSES}, not {status!r}")
        self.request("PATCH", f"/goals/{goal_id}", json={"status": status})
