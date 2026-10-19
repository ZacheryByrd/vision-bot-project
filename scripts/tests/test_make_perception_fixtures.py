"""Tests for scripts/make_perception_fixtures.py (runs the Python detectors; no ROS)."""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from make_perception_fixtures import PERSON_IMAGE, main  # noqa: E402


def test_writes_cases_and_reference(tmp_path):
    main(tmp_path)
    reference = json.loads((tmp_path / "reference.json").read_text())
    assert len(reference["hsv"]) >= 11
    for case in reference["hsv"]:
        assert (tmp_path / case["image"]).exists()
        assert len(case["expected"]) == 5
    centered = next(c for c in reference["hsv"] if c["image"] == "hsv_centered.png")
    assert centered["expected"][0] == 1.0


def test_writes_dnn_cases_with_the_person_detected(tmp_path):
    main(tmp_path)
    dnn = json.loads((tmp_path / "reference.json").read_text())["dnn"]
    assert len(dnn) >= 4
    for case in dnn:
        assert {"image", "target_class_id", "confidence_threshold", "expected"} <= case.keys()
    person = next(c for c in dnn if c["image"] == PERSON_IMAGE and c["target_class_id"] == 15
                  and c["confidence_threshold"] == 0.5)
    assert person["expected"][0] == 1.0 and person["expected"][4] > 0.5
