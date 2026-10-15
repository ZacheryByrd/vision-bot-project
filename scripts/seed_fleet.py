#!/usr/bin/env python3
"""Admin seed step for fleet_bridge (DECISIONS, 2026-10-14): run once per fleet.

Creates the two simulated robots (serial numbers VB-SIM-1 and VB-SIM-2) if
they are missing and, with --bridge-user, an operator account for the bridge.
The bridge password is generated here and written only to --env-file (git
ignores .env); it is never printed. Re-running is safe: existing robots are
reused, and an account whose password is already in the env file is checked
by logging in with it.

Env: FLEET_ADMIN_USER, FLEET_ADMIN_PASSWORD (an admin account), and
FLEET_API_BASE_URL (default http://localhost:8000/api/v1).

  python3 scripts/seed_fleet.py --bridge-user vision-bot-bridge@example.com \
      --env-file ros2_ws/src/fleet_bridge/.env \
      --bridge-api-url http://host.docker.internal:8001/api/v1
"""

import argparse
import os
import secrets
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "fleet_bridge"))

from fleet_bridge.client import FleetAuthError, FleetClient, FleetError, UrllibSession  # noqa: E402

ROBOTS = (("VB-SIM-1", "robot1"), ("VB-SIM-2", "robot2"))


def read_env(path):
    values = {}
    if path.exists():
        for line in path.read_text().splitlines():
            if "=" in line and not line.lstrip().startswith("#"):
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()
    return values


def bridge_account(base_url, user, env_file, bridge_api_url, admin):
    existing = read_env(env_file)
    if existing.get("FLEET_USER") == user and existing.get("FLEET_PASSWORD"):
        bridge = FleetClient(base_url, user, existing["FLEET_PASSWORD"], UrllibSession())
        bridge.login()
        return bridge, "already set up"
    password = secrets.token_urlsafe(18)
    try:
        admin.request("POST", "/users", json={"email": user, "full_name": "vision_bot fleet bridge",
                                              "password": password, "role_name": "operator"})
    except FleetError as err:
        if err.status == 409:
            sys.exit(f"{user} already exists but its password is not in {env_file}; "
                     "reset it in the fleet admin UI or choose another --bridge-user")
        raise
    env_file.write_text("# Written by scripts/seed_fleet.py; git ignores this file.\n"
                        f"FLEET_API_BASE_URL={bridge_api_url or base_url}\n"
                        f"FLEET_USER={user}\nFLEET_PASSWORD={password}\n")
    env_file.chmod(0o600)
    bridge = FleetClient(base_url, user, password, UrllibSession())
    bridge.login()
    return bridge, f"created; credentials written to {env_file} (password not shown)"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bridge-user", help="email of the bridge's operator account")
    parser.add_argument("--env-file", type=Path, help="where to write the bridge credentials")
    parser.add_argument("--bridge-api-url",
                        help="API URL as the bridge sees it (default: FLEET_API_BASE_URL)")
    args = parser.parse_args()
    if bool(args.bridge_user) != bool(args.env_file):
        parser.error("--bridge-user and --env-file go together")
    base_url = os.environ.get("FLEET_API_BASE_URL", "http://localhost:8000/api/v1")
    try:
        admin = FleetClient(base_url, os.environ["FLEET_ADMIN_USER"],
                            os.environ["FLEET_ADMIN_PASSWORD"], UrllibSession())
    except KeyError as missing:
        sys.exit(f"set {missing.args[0]}")

    try:
        for serial, name in ROBOTS:
            robot = admin.register_robot(serial, name)
            print(f"{serial} ({name}): fleet robot {robot['id']}")
        if args.bridge_user:
            bridge, outcome = bridge_account(base_url, args.bridge_user, args.env_file,
                                             args.bridge_api_url, admin)
            role = bridge.request("GET", "/auth/me")["role"]["name"]
            print(f"bridge account {args.bridge_user}: {outcome}; role {role}")
            if role != "operator":
                print(f"warning: the bridge account is {role}, not operator")
    except FleetAuthError as err:
        sys.exit(f"login refused: {err}")


if __name__ == "__main__":
    main()
