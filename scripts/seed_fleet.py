#!/usr/bin/env python3
"""Admin seed step for fleet_bridge and the dashboard (DECISIONS, 2026-10-14): run once per fleet.

Creates the two simulated robots (serial numbers VB-SIM-1 and VB-SIM-2) if
they are missing and, for each --account, an operator account (the bridge and
the dashboard each get their own). Each password is generated here and written
only to that account's env file (git ignores .env and .env.local); it is never
printed. The file's FLEET_* lines are replaced and every other line is kept.
Re-running is safe: existing robots are reused, and an account whose password
is already in its env file is checked by logging in with it.

Env: FLEET_ADMIN_USER, FLEET_ADMIN_PASSWORD (an admin account), and
FLEET_API_BASE_URL (default http://localhost:8000/api/v1).

  python3 scripts/seed_fleet.py \
      --account vision-bot-bridge@example.com ros2_ws/src/fleet_bridge/.env \
          http://host.docker.internal:8001/api/v1 \
      --account vision-bot-dashboard@example.com dashboard/.env.local \
          http://localhost:8001/api/v1
"""

import argparse
import os
import secrets
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "fleet_bridge"))

from fleet_bridge.client import FleetAuthError, FleetClient, FleetError, UrllibSession  # noqa: E402

ROBOTS = (("VB-SIM-1", "robot1"), ("VB-SIM-2", "robot2"))
KEYS = ("FLEET_API_BASE_URL", "FLEET_USER", "FLEET_PASSWORD")


def read_env(path):
    values = {}
    if path.exists():
        for line in path.read_text().splitlines():
            if "=" in line and not line.lstrip().startswith("#"):
                key, value = line.split("=", 1)
                values[key.strip()] = value.strip()
    return values


def merge_env(text, values):
    """text with the lines for values' keys replaced (or appended); other lines kept."""
    kept = [line for line in text.splitlines()
            if line.split("=", 1)[0].strip() not in values
            and line != "# Fleet API account, written by scripts/seed_fleet.py"]
    return "\n".join(kept + ["# Fleet API account, written by scripts/seed_fleet.py"]
                     + [f"{k}={v}" for k, v in values.items()]) + "\n"


def operator_account(base_url, user, env_file, account_api_url, admin):
    existing = read_env(env_file)
    if existing.get("FLEET_USER") == user and existing.get("FLEET_PASSWORD"):
        account = FleetClient(base_url, user, existing["FLEET_PASSWORD"], UrllibSession())
        try:
            account.login()
            return account, "already set up"
        except FleetAuthError:
            pass
        # The env file has credentials the fleet no longer knows (for example a
        # test stack that was reset): recreate the account with that password.
        try:
            admin.request("POST", "/users", json={
                "email": user, "full_name": user.split("@")[0],
                "password": existing["FLEET_PASSWORD"], "role_name": "operator"})
        except FleetError as err:
            if err.status == 409:
                sys.exit(f"{user} exists with a password other than the one in {env_file}; "
                         "reset it in the fleet admin UI or choose another account")
            raise
        account.login()
        return account, "recreated with the password in its env file (the fleet had been reset)"
    password = secrets.token_urlsafe(18)
    try:
        admin.request("POST", "/users", json={"email": user, "full_name": user.split("@")[0],
                                              "password": password, "role_name": "operator"})
    except FleetError as err:
        if err.status == 409:
            sys.exit(f"{user} already exists but its password is not in {env_file}; "
                     "reset it in the fleet admin UI or choose another account")
        raise
    text = env_file.read_text() if env_file.exists() else ""
    env_file.write_text(merge_env(text, dict(zip(KEYS, (account_api_url, user, password)))))
    env_file.chmod(0o600)
    account = FleetClient(base_url, user, password, UrllibSession())
    account.login()
    return account, f"created; credentials written to {env_file} (password not shown)"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--account", nargs=3, action="append", default=[],
                        metavar=("EMAIL", "ENV_FILE", "API_URL"),
                        help="an operator account, the env file for its credentials, and the "
                             "API URL as that client sees it")
    args = parser.parse_args()
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
        for user, env_file, api_url in args.account:
            account, outcome = operator_account(base_url, user, Path(env_file), api_url, admin)
            role = account.request("GET", "/auth/me")["role"]["name"]
            print(f"account {user}: {outcome}; role {role}")
            if role != "operator":
                print(f"warning: {user} is {role}, not operator")
    except FleetAuthError as err:
        sys.exit(f"login refused: {err}")


if __name__ == "__main__":
    main()
