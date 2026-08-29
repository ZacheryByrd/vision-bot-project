"""Unit tests for the pure control law in motor_control_node.py.

No rclpy, no publishers/subscribers -- compute_tracking_cmd is a plain
function of (offset_x, area_fraction) -> (linear_x, angular_z), so these
run in milliseconds with plain pytest:
    colcon test --packages-select vision_bot
or, from this package's root:
    pytest test/test_motor_control_logic.py
"""
import pytest

from vision_bot.motor_control_node import compute_tracking_cmd


DEFAULTS = dict(
    angular_gain=1.0,
    base_linear_speed=0.15,
    offset_deadband=0.05,
    stop_area_fraction=0.15,
)


def cmd(offset_x, area_fraction, **overrides):
    params = {**DEFAULTS, **overrides}
    return compute_tracking_cmd(offset_x, area_fraction, **params)


def test_centered_target_far_away_drives_straight():
    # area_fraction=0.0 (target not yet visibly close) is the only case
    # where the proximity_factor curve is exactly 1.0 -- any nonzero
    # area_fraction already pulls speed down a little, by design.
    linear_x, angular_z = cmd(offset_x=0.0, area_fraction=0.0)
    assert angular_z == 0.0
    assert linear_x == pytest.approx(DEFAULTS["base_linear_speed"], abs=1e-6)


def test_target_to_the_right_turns_right():
    # offset_x > 0 means the target is right of center; angular.z should be
    # negative (turn right, per the sign convention in the docstring).
    _, angular_z = cmd(offset_x=0.5, area_fraction=0.01)
    assert angular_z < 0


def test_target_to_the_left_turns_left():
    _, angular_z = cmd(offset_x=-0.5, area_fraction=0.01)
    assert angular_z > 0


def test_offset_within_deadband_is_ignored():
    linear_x, angular_z = cmd(offset_x=0.02, area_fraction=0.0, offset_deadband=0.05)
    assert angular_z == 0.0
    assert linear_x == pytest.approx(DEFAULTS["base_linear_speed"], abs=1e-6)


def test_offset_just_outside_deadband_is_not_ignored():
    _, angular_z = cmd(offset_x=0.06, area_fraction=0.01, offset_deadband=0.05)
    assert angular_z != 0.0


def test_stops_once_target_reaches_stop_area_fraction():
    linear_x, _ = cmd(offset_x=0.0, area_fraction=0.15, stop_area_fraction=0.15)
    assert linear_x == 0.0


def test_stays_stopped_well_past_stop_area_fraction():
    linear_x, _ = cmd(offset_x=0.0, area_fraction=0.9, stop_area_fraction=0.15)
    assert linear_x == 0.0


def test_still_allowed_to_turn_while_stopped_near_target():
    # This is the whole point of the stop-distance fix: linear.x holds at
    # zero, but angular.z still tracks the target so it doesn't drift
    # off-center while parked.
    linear_x, angular_z = cmd(offset_x=0.3, area_fraction=0.5, stop_area_fraction=0.15)
    assert linear_x == 0.0
    assert angular_z < 0


def test_slows_down_approaching_stop_threshold():
    linear_x_far, _ = cmd(offset_x=0.0, area_fraction=0.01, stop_area_fraction=0.15)
    linear_x_near, _ = cmd(offset_x=0.0, area_fraction=0.13, stop_area_fraction=0.15)
    assert 0.0 < linear_x_near < linear_x_far


def test_large_offset_slows_forward_speed():
    linear_x_centered, _ = cmd(offset_x=0.0, area_fraction=0.01)
    linear_x_off_center, _ = cmd(offset_x=0.8, area_fraction=0.01)
    assert linear_x_off_center < linear_x_centered
    assert linear_x_off_center >= 0.0
