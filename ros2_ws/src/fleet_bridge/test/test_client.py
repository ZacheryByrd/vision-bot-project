"""Tests for fleet_bridge/client.py with a fake HTTP session (no ROS, no network).

Run: python3 -m pytest ros2_ws/src/fleet_bridge/test
"""

import json
import logging
import sys
from pathlib import Path
from urllib.parse import parse_qs, urlsplit

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_bridge.client import (  # noqa: E402
    FleetAuthError,
    FleetClient,
    FleetError,
    FleetUnavailable,
    Goal,
    HttpResponse,
)

BASE = "http://fleet.test/api/v1"


def reply(status, body=None):
    return HttpResponse(status, b"" if body is None else json.dumps(body).encode())


def tokens(n):
    return reply(200, {"access_token": f"access-{n}", "refresh_token": f"refresh-{n}",
                       "token_type": "bearer"})


class FakeSession:
    """Answers each request with the next queued reply for its 'METHOD /path'."""

    def __init__(self):
        self.queues = {}
        self.calls = []

    def on(self, method, path, *replies):
        self.queues.setdefault(f"{method} {path}", []).extend(replies)
        return self

    def request(self, method, url, *, headers, json=None, form=None, timeout):
        parts = urlsplit(url)
        path = parts.path[len("/api/v1"):]
        self.calls.append({"method": method, "path": path, "query": parse_qs(parts.query),
                           "headers": dict(headers), "json": json, "form": form})
        queue = self.queues.get(f"{method} {path}")
        if not queue:
            raise AssertionError(f"unexpected request {method} {path}")
        answer = queue.pop(0)
        if isinstance(answer, Exception):
            raise answer
        return answer

    def paths(self):
        return [f"{c['method']} {c['path']}" for c in self.calls]


def client(session, **known_robots):
    c = FleetClient(BASE, "bridge@example.com", "s3cret-pw", session)
    c._robot_ids.update(known_robots)     # skip resolution where a test does not cover it
    return c


def test_login_stores_tokens():
    s = FakeSession().on("POST", "/auth/login", tokens(1))
    c = client(s)
    c.login()
    assert s.calls[0]["form"] == {"username": "bridge@example.com", "password": "s3cret-pw"}
    assert (c._access, c._refresh) == ("access-1", "refresh-1")


def test_login_with_bad_password_raises_auth_error():
    s = FakeSession().on("POST", "/auth/login", reply(401, {"detail": "Incorrect email"}))
    with pytest.raises(FleetAuthError):
        client(s).login()


def test_calls_send_the_access_token():
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("POST", "/telemetry/robots/7", reply(201, {}))
    client(s, SIM1=7).post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)
    assert s.calls[1]["headers"]["Authorization"] == "Bearer access-1"


def test_expired_token_refreshes_once_and_retries():
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("POST", "/telemetry/robots/7", reply(401), reply(201, {})) \
        .on("POST", "/auth/refresh", tokens(2))
    client(s, SIM1=7).post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)
    assert s.paths() == ["POST /auth/login", "POST /telemetry/robots/7", "POST /auth/refresh",
                         "POST /telemetry/robots/7"]
    assert s.calls[2]["json"] == {"refresh_token": "refresh-1"}
    assert s.calls[3]["headers"]["Authorization"] == "Bearer access-2"


def test_rotated_refresh_token_is_stored():
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("POST", "/telemetry/robots/7", reply(401), reply(201, {}), reply(401), reply(201, {})) \
        .on("POST", "/auth/refresh", tokens(2), tokens(3))
    c = client(s, SIM1=7)
    c.post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)
    assert c._refresh == "refresh-2"
    c.post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)
    refreshes = [call["json"] for call in s.calls if call["path"] == "/auth/refresh"]
    assert refreshes == [{"refresh_token": "refresh-1"}, {"refresh_token": "refresh-2"}]


def test_refresh_failure_triggers_relogin():
    s = FakeSession().on("POST", "/auth/login", tokens(1), tokens(2)) \
        .on("POST", "/telemetry/robots/7", reply(401), reply(201, {})) \
        .on("POST", "/auth/refresh", reply(401, {"detail": "expired"}))
    client(s, SIM1=7).post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)
    assert s.paths() == ["POST /auth/login", "POST /telemetry/robots/7", "POST /auth/refresh",
                         "POST /auth/login", "POST /telemetry/robots/7"]
    assert s.calls[4]["headers"]["Authorization"] == "Bearer access-2"


def test_still_unauthorized_after_relogin_raises_without_looping():
    s = FakeSession().on("POST", "/auth/login", tokens(1), tokens(2)) \
        .on("POST", "/telemetry/robots/7", reply(401), reply(401)) \
        .on("POST", "/auth/refresh", reply(401))
    with pytest.raises(FleetAuthError):
        client(s, SIM1=7).post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)
    assert len(s.calls) == 5


def test_api_down_raises_fleet_unavailable_not_crash():
    s = FakeSession().on("POST", "/auth/login", ConnectionRefusedError(111, "refused"))
    with pytest.raises(FleetUnavailable):
        client(s).login()
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("POST", "/telemetry/robots/7", TimeoutError("timed out"))
    with pytest.raises(FleetUnavailable):
        client(s, SIM1=7).post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)


def test_server_error_raises_fleet_error_with_status():
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("PATCH", "/goals/5", reply(422, {"detail": "bad status"}))
    with pytest.raises(FleetError) as err:
        client(s).report_goal_status("5", "succeeded")
    assert err.value.status == 422


@pytest.fixture
def client_log(caplog):
    """Capture the client's log records directly: ROS's launch_testing pytest plugin
    installs a logger class that does not propagate to the root logger."""
    logger = logging.getLogger("fleet_bridge.client")
    level = logger.level
    logger.addHandler(caplog.handler)
    logger.setLevel(logging.DEBUG)
    yield caplog
    logger.removeHandler(caplog.handler)
    logger.setLevel(level)


def test_tokens_never_logged(client_log):
    # first call: login, 401, refresh, retry; second call: 401, refresh fails, login, retry
    s = FakeSession().on("POST", "/auth/login", tokens(1), tokens(3)) \
        .on("POST", "/telemetry/robots/7", reply(401), reply(201, {}), reply(401), reply(201, {})) \
        .on("POST", "/auth/refresh", tokens(2), reply(401))
    c = client(s, SIM1=7)
    c.post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)
    c.post_telemetry("SIM1", (1.0, 2.0, 0.5), "idle", 80.0)
    caplog = client_log
    assert len(caplog.records) >= 3, "the client should log login, refresh and re-login"
    text = caplog.text + repr(c)
    for secret in ("access-1", "access-2", "access-3", "refresh-1", "refresh-2", "refresh-3",
                   "s3cret-pw"):
        assert secret not in text


def test_register_robot_resolves_its_serial_across_pages():
    page1 = {"items": [{"id": i, "serial_number": f"OTHER-{i}"} for i in range(1, 101)],
             "total": 102, "skip": 0, "limit": 100}
    page2 = {"items": [{"id": 101, "serial_number": "OTHER-101"},
                       {"id": 102, "serial_number": "SIM2", "name": "robot2"}],
             "total": 102, "skip": 100, "limit": 100}
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("GET", "/robots", reply(200, page1), reply(200, page2))
    c = client(s)
    robot = c.register_robot("SIM2", "robot2")
    assert robot["id"] == 102 and c._robot_ids["SIM2"] == 102
    assert [call["query"]["skip"] for call in s.calls[1:]] == [["0"], ["100"]]


def test_register_robot_creates_it_when_missing():
    empty = {"items": [], "total": 0, "skip": 0, "limit": 100}
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("GET", "/robots", reply(200, empty)) \
        .on("POST", "/robots", reply(201, {"id": 9, "serial_number": "SIM1"}))
    c = client(s)
    assert c.register_robot("SIM1", "robot1")["id"] == 9
    assert s.calls[2]["json"]["serial_number"] == "SIM1"
    assert s.calls[2]["json"]["name"] == "robot1"


def test_register_robot_without_create_rights_explains_the_seed_step():
    empty = {"items": [], "total": 0, "skip": 0, "limit": 100}
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("GET", "/robots", reply(200, empty)) \
        .on("POST", "/robots", reply(403, {"detail": "Insufficient permissions"}))
    with pytest.raises(FleetError, match="seed"):
        client(s).register_robot("SIM1", "robot1")


def test_post_telemetry_sends_pose_state_and_battery():
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("POST", "/telemetry/robots/7", reply(201, {}))
    client(s, SIM1=7).post_telemetry("SIM1", (1.5, -2.0, 0.25), "navigating", 73.5)
    assert s.calls[1]["json"] == {"pose_x": 1.5, "pose_y": -2.0, "pose_yaw": 0.25,
                                  "nav_state": "navigating", "battery_level": 73.5,
                                  "battery_simulated": True}


def test_unregistered_robot_is_an_error_not_a_request():
    with pytest.raises(FleetError, match="register_robot"):
        client(FakeSession()).post_telemetry("SIM9", (0.0, 0.0, 0.0), "idle", 50.0)


def test_fetch_pending_goal_returns_the_oldest_or_none():
    goals = [{"id": 12, "x": 3.0, "y": -0.5, "yaw": 1.57, "status": "pending"},
             {"id": 11, "x": 1.0, "y": 2.0, "yaw": 0.0, "status": "pending"}]
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("GET", "/robots/7/goals", reply(200, goals), reply(200, []))
    c = client(s, SIM1=7)
    assert c.fetch_pending_goal("SIM1") == Goal("11", 1.0, 2.0, 0.0)
    assert s.calls[1]["query"] == {"status": ["pending"]}
    assert c.fetch_pending_goal("SIM1") is None


def test_report_goal_status_patches_the_goal():
    s = FakeSession().on("POST", "/auth/login", tokens(1)) \
        .on("PATCH", "/goals/11", reply(200, {"id": 11, "status": "accepted"}))
    client(s).report_goal_status("11", "accepted")
    assert s.calls[1]["json"] == {"status": "accepted"}


def test_report_goal_status_rejects_unknown_status():
    with pytest.raises(ValueError):
        client(FakeSession()).report_goal_status("11", "aborted")
