"""Tests for scripts/make_control_fixtures.py (runs the Python control law; no ROS graph)."""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from make_control_fixtures import main  # noqa: E402


def test_writes_reference_grid(tmp_path):
    main(tmp_path)
    cases = json.loads((tmp_path / "reference.json").read_text())["cases"]
    assert len(cases) >= 200
    assert all(len(c["expected"]) == 2 for c in cases)
    defaults = {"angular_gain": 1.0, "base_linear_speed": 0.15, "offset_deadband": 0.05,
                "stop_area_fraction": 0.15}
    centered = next(c for c in cases if c["offset_x"] == 0.0 and c["area_fraction"] == 0.0
                    and c["params"] == defaults)
    assert centered["expected"][0] == 0.15
    assert centered["expected"][1] == 0.0 and math.copysign(1.0, centered["expected"][1]) == -1.0
