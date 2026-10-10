"""Tests for the pure phase evaluation in scripts/mux_check.py (no ROS needed)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from mux_check import Phase, Sample, evaluate_phase  # noqa: E402

LINE = Phase(name="line", duration_s=3.0, inputs={"cmd_vel_line": (0.2, 0.0)},
             expect=(0.2, 0.0))
STOP = Phase(name="stop", duration_s=3.0, inputs={}, expect=(0.0, 0.0))


def test_phase_passes_when_command_and_motion_match():
    cmd = [Sample(1.0, 0.2, 0.0), Sample(2.5, 0.2, 0.0)]
    odom = [Sample(2.0, 0.19, 0.01), Sample(2.8, 0.21, -0.01)]
    result = evaluate_phase(LINE, t0=0.0, cmd=cmd, odom=odom)
    assert result.ok
    assert result.cmd == (0.2, 0.0)


def test_only_the_second_half_of_the_phase_counts():
    # The mux may still be forwarding the previous phase's command just
    # after a switch; only samples after the midpoint are judged.
    cmd = [Sample(0.2, 0.0, 0.8), Sample(2.0, 0.2, 0.0)]
    odom = [Sample(0.3, 0.0, 0.8), Sample(2.2, 0.2, 0.0)]
    result = evaluate_phase(LINE, t0=0.0, cmd=cmd, odom=odom)
    assert result.ok


def test_wrong_command_fails():
    cmd = [Sample(2.0, 0.0, 0.8)]
    odom = [Sample(2.2, 0.2, 0.0)]
    assert not evaluate_phase(LINE, t0=0.0, cmd=cmd, odom=odom).ok


def test_robot_still_moving_fails_even_without_commands():
    # The runaway case: nothing on cmd_vel, but the drive keeps the last
    # command and odometry shows the robot moving.
    odom = [Sample(2.0, 0.2, 0.0), Sample(2.9, 0.2, 0.0)]
    result = evaluate_phase(STOP, t0=0.0, cmd=[], odom=odom)
    assert not result.ok
    assert result.cmd is None


def test_stop_phase_needs_an_explicit_zero_command():
    odom = [Sample(2.0, 0.0, 0.0)]
    assert not evaluate_phase(STOP, t0=0.0, cmd=[], odom=odom).ok
    assert evaluate_phase(STOP, t0=0.0, cmd=[Sample(2.5, 0.0, 0.0)], odom=odom).ok


def test_motion_is_averaged_and_compared_with_tolerance():
    cmd = [Sample(2.0, 0.2, 0.0)]
    odom = [Sample(1.6, 0.10, 0.0), Sample(2.4, 0.10, 0.0)]  # mean 0.10, too slow
    assert not evaluate_phase(LINE, t0=0.0, cmd=cmd, odom=odom).ok


def test_phase_offset_by_t0():
    cmd = [Sample(12.0, 0.2, 0.0)]
    odom = [Sample(12.5, 0.2, 0.0)]
    assert evaluate_phase(LINE, t0=10.0, cmd=cmd, odom=odom).ok
    assert not evaluate_phase(LINE, t0=0.0, cmd=cmd, odom=odom).ok


# --- Gate 3: Nav2 as an input ("expect=None" means "Nav2 is driving") -----

NAV = Phase(name="nav driving", duration_s=4.0, inputs={}, expect=None)


def test_nav_phase_passes_when_commands_flow_and_robot_moves():
    cmd = [Sample(2.5, 0.22, -0.1), Sample(3.5, 0.24, 0.05)]
    odom = [Sample(2.2, 0.2, 0.0), Sample(3.8, 0.21, 0.0)]
    assert evaluate_phase(NAV, t0=0.0, cmd=cmd, odom=odom).ok


def test_nav_phase_fails_without_commands():
    odom = [Sample(2.2, 0.2, 0.0)]
    assert not evaluate_phase(NAV, t0=0.0, cmd=[], odom=odom).ok


def test_nav_phase_fails_on_zero_commands():
    # The idle input's zeros are not Nav2 driving.
    cmd = [Sample(2.5, 0.0, 0.0), Sample(3.5, 0.0, 0.0)]
    odom = [Sample(2.2, 0.0, 0.0)]
    assert not evaluate_phase(NAV, t0=0.0, cmd=cmd, odom=odom).ok


def test_nav_phase_fails_when_robot_does_not_move():
    cmd = [Sample(2.5, 0.2, 0.0)]
    odom = [Sample(2.2, 0.0, 0.0), Sample(3.8, 0.0, 0.0)]
    assert not evaluate_phase(NAV, t0=0.0, cmd=cmd, odom=odom).ok


def test_turning_in_place_counts_as_moving():
    cmd = [Sample(2.5, 0.0, 0.6)]
    odom = [Sample(2.2, 0.0, 0.6)]
    assert evaluate_phase(NAV, t0=0.0, cmd=cmd, odom=odom).ok
