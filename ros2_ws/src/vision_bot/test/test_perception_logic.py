"""Unit tests for the pure detection function in perception_node.py.

Runs against synthetic images built with plain numpy/OpenCV -- no camera,
no rclpy node. Same run instructions as test_motor_control_logic.py.
"""
import numpy as np
import pytest

from vision_bot.perception_node import detect_target


WIDTH, HEIGHT = 320, 240
DEFAULTS = dict(hue_low=0, hue_high=10, sat_low=120, val_low=80, min_area=200)

# Pure red in BGR -> hue 0, saturation 255, value 255 in HSV: solidly
# inside the default threshold.
RED_BGR = (0, 0, 255)


def blank_frame():
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    frame[:] = (40, 40, 40)  # dark grey background, well outside the red threshold
    return frame


def draw_box(frame, cx, cy, size, color=RED_BGR):
    import cv2
    half = size // 2
    cv2.rectangle(frame, (cx - half, cy - half), (cx + half, cy + half), color, -1)
    return frame


def detect(frame, **overrides):
    params = {**DEFAULTS, **overrides}
    return detect_target(frame, **params)


def test_empty_frame_no_detection():
    detected, offset_x, offset_y, area_fraction, _ = detect(blank_frame())
    assert detected is False
    assert offset_x == 0.0
    assert area_fraction == 0.0


def test_centered_box_detected_near_zero_offset():
    frame = draw_box(blank_frame(), WIDTH // 2, HEIGHT // 2, size=60)
    detected, offset_x, offset_y, area_fraction, _ = detect(frame)
    assert detected is True
    assert offset_x == pytest.approx(0.0, abs=0.02)
    assert offset_y == pytest.approx(0.0, abs=0.02)
    assert area_fraction > 0


def test_box_left_of_center_gives_negative_offset_x():
    frame = draw_box(blank_frame(), WIDTH // 4, HEIGHT // 2, size=40)
    detected, offset_x, _, _, _ = detect(frame)
    assert detected is True
    assert offset_x < 0


def test_box_right_of_center_gives_positive_offset_x():
    frame = draw_box(blank_frame(), 3 * WIDTH // 4, HEIGHT // 2, size=40)
    detected, offset_x, _, _, _ = detect(frame)
    assert detected is True
    assert offset_x > 0


def test_box_above_center_gives_negative_offset_y():
    frame = draw_box(blank_frame(), WIDTH // 2, HEIGHT // 4, size=40)
    detected, _, offset_y, _, _ = detect(frame)
    assert detected is True
    assert offset_y < 0


def test_larger_box_gives_larger_area_fraction():
    small = draw_box(blank_frame(), WIDTH // 2, HEIGHT // 2, size=20)
    large = draw_box(blank_frame(), WIDTH // 2, HEIGHT // 2, size=100)
    _, _, _, area_small, _ = detect(small)
    _, _, _, area_large, _ = detect(large)
    assert area_large > area_small


def test_box_smaller_than_min_area_is_ignored():
    frame = draw_box(blank_frame(), WIDTH // 2, HEIGHT // 2, size=5)  # ~25px^2
    detected, _, _, _, _ = detect(frame, min_area=200)
    assert detected is False


def test_non_matching_color_is_ignored():
    # Solid blue box -- well outside the default red hue threshold.
    frame = draw_box(blank_frame(), WIDTH // 2, HEIGHT // 2, size=60, color=(255, 0, 0))
    detected, _, _, _, _ = detect(frame)
    assert detected is False


def test_debug_frame_has_same_shape_as_input():
    frame = draw_box(blank_frame(), WIDTH // 2, HEIGHT // 2, size=60)
    _, _, _, _, debug_frame = detect(frame)
    assert debug_frame.shape == frame.shape
