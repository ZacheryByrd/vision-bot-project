#!/usr/bin/env python3
"""Reference fixture for the C++ control law (vision_bot_control).

Writes reference.json: a grid of (offset_x, area_fraction, params) and what
compute_tracking_cmd in vision_bot/motor_control_node.py returns for each.
The C++ gtest computes the same cases and compares, sign of zero included
(Python's -gain * 0.0 is -0.0, and that is what reaches /cmd_vel).

  python3 scripts/make_control_fixtures.py \
      ros2_ws/src/vision_bot_control/test/fixtures
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "vision_bot"))

from vision_bot.motor_control_node import compute_tracking_cmd  # noqa: E402

OFFSETS = [-1.5, -1.0, -0.5, -0.06, -0.05, -0.049, 0.0, 0.02, 0.05, 0.3, 0.8, 1.0, 1.5]
AREAS = [0.0, 0.01, 0.1, 0.13, 0.1499, 0.15, 0.5, 0.9, 1.0]
DEFAULTS = {"angular_gain": 1.0, "base_linear_speed": 0.15, "offset_deadband": 0.05,
            "stop_area_fraction": 0.15}
# line_follow_launch.py's tuning: drive continuously, turn harder.
LINE_FOLLOW = dict(DEFAULTS, angular_gain=1.8, base_linear_speed=0.10, stop_area_fraction=1.0)
NO_STOP_AREA = dict(DEFAULTS, stop_area_fraction=0.0)  # never divides: area >= 0 always stops
NO_DEADBAND = dict(DEFAULTS, offset_deadband=0.0)


def make_cases():
    """(offset_x, area_fraction, params) for every reference case."""
    cases = [(o, a, p) for p in (DEFAULTS, LINE_FOLLOW) for o in OFFSETS for a in AREAS]
    cases += [(o, a, NO_STOP_AREA) for o in (-0.5, 0.0, 0.5) for a in AREAS]
    cases += [(o, a, NO_DEADBAND) for o in (-0.02, 0.0, 0.02) for a in (0.0, 0.1)]
    return cases


def main(out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    cases = []
    for offset_x, area_fraction, params in make_cases():
        linear_x, angular_z = compute_tracking_cmd(offset_x, area_fraction, **params)
        cases.append({"offset_x": offset_x, "area_fraction": area_fraction, "params": params,
                      "expected": [linear_x, angular_z]})
    (out_dir / "reference.json").write_text(json.dumps({"cases": cases}, indent=1) + "\n")
    print(f"{len(cases)} control cases written to {out_dir}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} OUT_DIR")
    main(sys.argv[1])
