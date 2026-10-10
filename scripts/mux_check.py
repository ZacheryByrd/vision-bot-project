#!/usr/bin/env python3
"""Check twist_mux arbitration end to end (Task 1 gate; reused at Gate 3).

Publishes velocity inputs on a fixed schedule of phases, records what
reaches the drive (cmd_vel) and how the robot actually moves (odom twist),
and judges each phase on its second half, after the mux has had time to
switch. A phase passes when every cmd_vel sample matches the expected
command and the mean odometry velocity matches it within tolerance.
A stop phase therefore needs an explicit zero command and a stopped robot.

Run inside the container with nav_sim.launch.py up:
    python3 scripts/mux_check.py            # exit 0 = all phases pass

With nav2.launch.py up, --nav-goal X Y sends a goal on goal_pose (what
RViz's "2D Goal Pose" tool publishes), waits until Nav2 is driving, then
checks that teleop overrides Nav2 and that Nav2 resumes when teleop stops:
    python3 scripts/mux_check.py --nav-goal -3.0 -0.5
"""

import argparse
import sys
import time
from dataclasses import dataclass, field

CMD_TOL = 0.01   # cmd_vel must equal the expected command
LIN_TOL = 0.05   # m/s, mean odometry linear velocity
ANG_TOL = 0.15   # rad/s, mean odometry angular velocity
MOVING_LIN = 0.03  # m/s: "Nav2 is driving" needs at least this mean speed...
MOVING_ANG = 0.1   # rad/s: ...or at least this mean turn rate


@dataclass(frozen=True)
class Sample:
    t: float     # seconds since the check started
    lin: float   # linear.x
    ang: float   # angular.z


@dataclass(frozen=True)
class Phase:
    name: str
    duration_s: float
    inputs: dict = field(default_factory=dict)   # topic -> (lin, ang)
    expect: tuple = (0.0, 0.0)                   # (lin, ang) at the drive


@dataclass(frozen=True)
class PhaseResult:
    name: str
    expect: tuple
    cmd: tuple      # last cmd_vel in the judged window, or None
    motion: tuple   # mean odometry (lin, ang) in the judged window, or None
    ok: bool


# With Nav2 running and a goal sent; expect=None means "Nav2 is driving".
NAV_PHASES = [
    Phase("nav driving", 4.0, {}, None),
    Phase("teleop over nav", 4.0, {"cmd_vel_teleop": (0.0, 0.8)}, (0.0, 0.8)),
    Phase("teleop released", 6.0, {}, None),
]

DEFAULT_PHASES = [
    Phase("idle", 2.0, {}, (0.0, 0.0)),
    Phase("line only", 3.0, {"cmd_vel_line": (0.2, 0.0)}, (0.2, 0.0)),
    Phase("line + teleop", 3.0,
          {"cmd_vel_line": (0.2, 0.0), "cmd_vel_teleop": (0.0, 0.8)}, (0.0, 0.8)),
    Phase("teleop released", 3.0, {"cmd_vel_line": (0.2, 0.0)}, (0.2, 0.0)),
    Phase("all inputs stopped", 3.0, {}, (0.0, 0.0)),
]


def evaluate_phase(phase, t0, cmd, odom):
    """Judge one phase that started at t0 from the recorded samples."""
    start = t0 + phase.duration_s / 2
    end = t0 + phase.duration_s
    cmd_w = [s for s in cmd if start <= s.t < end]
    odom_w = [s for s in odom if start <= s.t < end]

    last_cmd = (cmd_w[-1].lin, cmd_w[-1].ang) if cmd_w else None
    motion = None
    if odom_w:
        motion = (sum(s.lin for s in odom_w) / len(odom_w),
                  sum(s.ang for s in odom_w) / len(odom_w))

    if phase.expect is None:
        # Nav2 in control: its commands are not known in advance, so require
        # non-zero commands at the drive and a robot that is actually moving.
        cmd_ok = any(abs(s.lin) > CMD_TOL or abs(s.ang) > CMD_TOL for s in cmd_w)
        motion_ok = motion is not None and (
            abs(motion[0]) >= MOVING_LIN or abs(motion[1]) >= MOVING_ANG)
    else:
        exp_lin, exp_ang = phase.expect
        cmd_ok = bool(cmd_w) and all(
            abs(s.lin - exp_lin) <= CMD_TOL and abs(s.ang - exp_ang) <= CMD_TOL for s in cmd_w)
        motion_ok = motion is not None and (
            abs(motion[0] - exp_lin) <= LIN_TOL and abs(motion[1] - exp_ang) <= ANG_TOL)
    return PhaseResult(phase.name, phase.expect, last_cmd, motion, cmd_ok and motion_ok)


def _run(phases, namespace, nav_goal=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    def topic(name):
        return f"{namespace.rstrip('/')}/{name}" if namespace else name

    rclpy.init()
    node = rclpy.create_node("mux_check")
    cmd, odom = [], []
    start = time.monotonic()

    def now():
        return time.monotonic() - start

    node.create_subscription(
        Twist, topic("cmd_vel"),
        lambda m: cmd.append(Sample(now(), m.linear.x, m.angular.z)), 10)
    node.create_subscription(
        Odometry, topic("odom"),
        lambda m: odom.append(Sample(now(), m.twist.twist.linear.x, m.twist.twist.angular.z)),
        10)
    input_topics = sorted({t for p in phases for t in p.inputs})
    pubs = {t: node.create_publisher(Twist, topic(t), 10) for t in input_topics}
    if nav_goal is not None:
        from geometry_msgs.msg import PoseStamped
        goal_pub = node.create_publisher(PoseStamped, topic("goal_pose"), 10)

    # Let discovery settle before the clock that phases are judged on starts.
    settle = time.monotonic() + 2.0
    while time.monotonic() < settle:
        rclpy.spin_once(node, timeout_sec=0.05)
    if nav_goal is not None:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x, goal.pose.position.y = nav_goal
        goal.pose.orientation.w = 1.0
        goal_pub.publish(goal)
        # Start judging once Nav2 is actually driving.
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline and not any(
                abs(s.lin) > CMD_TOL or abs(s.ang) > CMD_TOL for s in cmd[-5:]):
            rclpy.spin_once(node, timeout_sec=0.05)
    cmd.clear()
    odom.clear()
    start = time.monotonic()

    starts, t = [], 0.0
    for p in phases:
        starts.append(t)
        t += p.duration_s
    total = t

    def publish_inputs():
        elapsed = now()
        for p, t0 in zip(phases, starts):
            if t0 <= elapsed < t0 + p.duration_s:
                for name, (lin, ang) in p.inputs.items():
                    msg = Twist()
                    msg.linear.x, msg.angular.z = lin, ang
                    pubs[name].publish(msg)

    node.create_timer(0.1, publish_inputs)
    while now() < total:
        rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    rclpy.shutdown()
    return [evaluate_phase(p, t0, cmd, odom) for p, t0 in zip(phases, starts)]


def _fmt(pair):
    return "none" if pair is None else f"({pair[0]:+.2f}, {pair[1]:+.2f})"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--namespace", default="", help="robot namespace, e.g. /robot1")
    parser.add_argument("--nav-goal", nargs=2, type=float, metavar=("X", "Y"),
                        help="send this map-frame goal to Nav2 and run the Nav2 phases")
    args = parser.parse_args()

    if args.nav_goal:
        results = _run(NAV_PHASES, args.namespace, nav_goal=tuple(args.nav_goal))
    else:
        results = _run(DEFAULT_PHASES, args.namespace)
    print(f"{'phase':<22}{'expected':<18}{'cmd_vel':<18}{'odom motion':<18}result")
    for r in results:
        expected = "Nav2 driving" if r.expect is None else _fmt(r.expect)
        print(f"{r.name:<22}{expected:<18}{_fmt(r.cmd):<18}{_fmt(r.motion):<18}"
              f"{'PASS' if r.ok else 'FAIL'}")
    ok = all(r.ok for r in results)
    print("mux_check:", "PASS" if ok else "FAIL")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
