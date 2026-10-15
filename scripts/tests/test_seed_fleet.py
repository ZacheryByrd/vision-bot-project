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
