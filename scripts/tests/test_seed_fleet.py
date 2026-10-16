"""Tests for the env-file merge in scripts/seed_fleet.py (no network)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from seed_fleet import merge_env  # noqa: E402

ACCOUNT = {"FLEET_API_BASE_URL": "http://h:8001/api/v1", "FLEET_USER": "u@example.com",
           "FLEET_PASSWORD": "p"}


def test_merge_keeps_other_lines_and_appends_the_account():
    text = "NEXT_PUBLIC_ROS_WS_URL=ws://localhost:9091\n# a comment\n"
    out = merge_env(text, ACCOUNT)
    assert out.splitlines()[:2] == ["NEXT_PUBLIC_ROS_WS_URL=ws://localhost:9091", "# a comment"]
    assert "FLEET_USER=u@example.com" in out.splitlines()


def test_merging_twice_replaces_rather_than_duplicates():
    once = merge_env("A=1\n", ACCOUNT)
    twice = merge_env(once, {**ACCOUNT, "FLEET_PASSWORD": "q"})
    assert twice.count("FLEET_PASSWORD=") == 1 and "FLEET_PASSWORD=q" in twice
    assert twice.count("written by scripts/seed_fleet.py") == 1
    assert twice.startswith("A=1\n")


class FakeAccount:
    """Stands in for FleetClient: login succeeds only for known (user, password) pairs."""
    known = set()

    def __init__(self, base_url, user, password, http):
        self.user, self.password = user, password

    def login(self):
        from fleet_bridge.client import FleetAuthError
        if (self.user, self.password) not in FakeAccount.known:
            raise FleetAuthError(401, "Incorrect email or password")


class FakeAdmin:
    def __init__(self):
        self.created = []

    def request(self, method, path, json=None):
        assert (method, path) == ("POST", "/users")
        self.created.append(json)
        FakeAccount.known.add((json["email"], json["password"]))
        return {"id": 1}


def test_account_in_env_file_but_missing_from_a_reset_fleet_is_recreated(tmp_path, monkeypatch):
    import seed_fleet
    monkeypatch.setattr(seed_fleet, "FleetClient", FakeAccount)
    FakeAccount.known = set()
    env = tmp_path / ".env"
    env.write_text("FLEET_USER=u@example.com\nFLEET_PASSWORD=kept-pw\n")
    admin = FakeAdmin()
    _, outcome = seed_fleet.operator_account("http://x/api/v1", "u@example.com", env, "http://x",
                                             admin)
    assert admin.created[0]["password"] == "kept-pw" and admin.created[0]["role_name"] == "operator"
    assert "recreated" in outcome
    assert env.read_text() == "FLEET_USER=u@example.com\nFLEET_PASSWORD=kept-pw\n"


def test_account_that_still_logs_in_is_left_alone(tmp_path, monkeypatch):
    import seed_fleet
    monkeypatch.setattr(seed_fleet, "FleetClient", FakeAccount)
    FakeAccount.known = {("u@example.com", "kept-pw")}
    env = tmp_path / ".env"
    env.write_text("FLEET_USER=u@example.com\nFLEET_PASSWORD=kept-pw\n")
    admin = FakeAdmin()
    _, outcome = seed_fleet.operator_account("http://x/api/v1", "u@example.com", env, "http://x",
                                             admin)
    assert admin.created == [] and outcome == "already set up"
