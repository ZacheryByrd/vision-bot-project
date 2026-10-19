#!/usr/bin/env python3
"""Reference fixtures for the C++ perception detectors (vision_bot_perception).

Writes test frames as PNGs plus reference.json: for each frame, the
parameters and what the Python detectors in vision_bot/perception_node.py
return for it. The C++ gtests run their detectors on the same PNGs and
compare. Each frame is read back from disk before the Python detector runs,
so both languages see exactly the same pixels.

  python3 scripts/make_perception_fixtures.py \
      ros2_ws/src/vision_bot_perception/test/fixtures
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws" / "src" / "vision_bot"))

from vision_bot.perception_node import detect_target  # noqa: E402

WIDTH, HEIGHT = 320, 240
BACKGROUND = (40, 40, 40)
RED, BLUE, YELLOW = (0, 0, 255), (255, 0, 0), (0, 255, 255)
HSV_DEFAULTS = dict(hue_low=0, hue_high=10, sat_low=120, val_low=80, min_area=200)
# vision_bot_nav's line-follow tuning for the yellow track.
YELLOW_PARAMS = dict(HSV_DEFAULTS, hue_low=25, hue_high=35, min_area=150)


def frame_with_boxes(*boxes):
    """A grey frame with filled boxes, each (cx, cy, size, bgr)."""
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    frame[:] = BACKGROUND
    for cx, cy, size, color in boxes:
        half = size // 2
        cv2.rectangle(frame, (cx - half, cy - half), (cx + half, cy + half), color, -1)
    return frame


def make_hsv_cases():
    """(name, frame, params) for the HSV detector."""
    cx, cy = WIDTH // 2, HEIGHT // 2
    return [
        ("hsv_empty", frame_with_boxes(), HSV_DEFAULTS),
        ("hsv_centered", frame_with_boxes((cx, cy, 60, RED)), HSV_DEFAULTS),
        ("hsv_left", frame_with_boxes((WIDTH // 4, cy, 40, RED)), HSV_DEFAULTS),
        ("hsv_right", frame_with_boxes((3 * WIDTH // 4, cy, 40, RED)), HSV_DEFAULTS),
        ("hsv_above", frame_with_boxes((cx, HEIGHT // 4, 40, RED)), HSV_DEFAULTS),
        ("hsv_small", frame_with_boxes((cx, cy, 20, RED)), HSV_DEFAULTS),
        ("hsv_large", frame_with_boxes((cx, cy, 100, RED)), HSV_DEFAULTS),
        ("hsv_below_min_area", frame_with_boxes((cx, cy, 5, RED)), HSV_DEFAULTS),
        ("hsv_blue", frame_with_boxes((cx, cy, 60, BLUE)), HSV_DEFAULTS),
        ("hsv_tie", frame_with_boxes((80, cy, 40, RED), (240, cy, 40, RED)), HSV_DEFAULTS),
        ("hsv_yellow", frame_with_boxes((cx + 30, cy + 20, 50, YELLOW)), YELLOW_PARAMS),
        ("hsv_off_corner", frame_with_boxes((10, 10, 30, RED)), HSV_DEFAULTS),
    ]


def as_expected(result):
    detected, offset_x, offset_y, area_fraction, confidence, _ = result
    return [1.0 if detected else 0.0, float(offset_x), float(offset_y),
            float(area_fraction), float(confidence)]


def main(out_dir):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    hsv = []
    for name, frame, params in make_hsv_cases():
        image = f"{name}.png"
        cv2.imwrite(str(out_dir / image), frame)
        on_disk = cv2.imread(str(out_dir / image))
        hsv.append({"image": image, "params": params,
                    "expected": as_expected(detect_target(on_disk, **params))})
    (out_dir / "reference.json").write_text(json.dumps({"hsv": hsv}, indent=1) + "\n")
    print(f"{len(hsv)} HSV cases written to {out_dir}")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} OUT_DIR")
    main(sys.argv[1])
