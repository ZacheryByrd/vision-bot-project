#!/usr/bin/env python3
"""Raw wheel odometry vs the EKF, scored against ground truth (Gate 2).

Needs nav_sim.launch.py running (it includes the EKF). Drives the
drift_loop route (two laps of a rectangle, 22.8 m) with RouteDriver and
records, at 10 Hz of sim time:
    raw wheel odometry   odom
    EKF output           odometry/filtered
    ground truth         ground_truth/pose
Each path is expressed relative to its own pose at the start of the route
(odometry starts wherever the robot was; ground truth is in the world
frame), then metrics.position_errors scores both estimates.

Writes benchmarks/results/odom_drift_<timestamp>.{csv,md,png}.
    python3 benchmarks/odom_drift.py --label "encoder odometry, no added noise"
"""

import argparse
import csv
import math
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from drive_route import RouteDriver, make_node
from metrics import position_errors, relative_to
from route import ROUTES, pose_from_odometry, route_length

RESULTS = Path(__file__).resolve().parent / "results"
SOURCES = {"truth": "ground_truth/pose", "odom": "odom", "ekf": "odometry/filtered"}


def _record(route_name, timeout_s):
    rclpy.init()
    node = make_node("odom_drift")
    latest = {}
    for key, topic in SOURCES.items():
        node.create_subscription(
            Odometry, topic,
            lambda msg, key=key: latest.__setitem__(key, pose_from_odometry(msg)),
            qos_profile_sensor_data)
    driver = RouteDriver(node, ROUTES[route_name])
    samples = []   # (t, truth, odom, ekf)

    def sample():
        if driver.pose is not None and not driver.done and len(latest) == len(SOURCES):
            t = node.get_clock().now().nanoseconds * 1e-9
            samples.append((t, latest["truth"], latest["odom"], latest["ekf"]))

    node.create_timer(0.1, sample)
    deadline = time.monotonic() + timeout_s
    while rclpy.ok() and not driver.done and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    failure = driver.failed or (None if driver.done else f"wall-clock timeout ({timeout_s} s)")
    node.destroy_node()
    rclpy.shutdown()
    return samples, failure


def _heading_error_deg(estimate, truth):
    return math.degrees(math.atan2(math.sin(estimate.yaw - truth.yaw),
                                   math.cos(estimate.yaw - truth.yaw)))


def _git_commit():
    """Short HEAD hash, flagged when the working tree has uncommitted changes.

    core.autocrlf=input: the repo is a Windows checkout (CRLF working files)
    seen by Linux git in the container, which would otherwise report every
    CRLF file as modified.
    """
    def git(*args):
        return subprocess.run(["git", "-c", "safe.directory=*", "-c", "core.autocrlf=input",
                               *args], capture_output=True, text=True, cwd=RESULTS.parent,
                              check=True).stdout.strip()
    try:
        head = git("rev-parse", "--short", "HEAD")
        dirty = git("status", "--porcelain", "--untracked-files=no")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    return f"{head} (+ uncommitted changes)" if dirty else head


def _plot(path, truth, odom, ekf, times, err_odom, err_ekf):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (a, b) = plt.subplots(1, 2, figsize=(12, 5.5))
    for poses, label, style in ((truth, "ground truth", "k-"), (odom, "raw odometry", "r--"),
                                (ekf, "EKF", "b-")):
        a.plot([p.x for p in poses], [p.y for p in poses], style, label=label, lw=1.4)
    a.set_aspect("equal")
    a.set_xlabel("x from route start (m)")
    a.set_ylabel("y from route start (m)")
    a.set_title("Paths, each relative to its own start pose")
    a.legend()
    a.grid(True, lw=0.3)
    b.plot(times, err_odom, "r--", label="raw odometry")
    b.plot(times, err_ekf, "b-", label="EKF")
    b.set_xlabel("sim time since route start (s)")
    b.set_ylabel("position error vs ground truth (m)")
    b.set_title("Position error over the route")
    b.legend()
    b.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", required=True,
                        help="odometry error model in effect, recorded in the summary")
    parser.add_argument("--route", default="drift_loop", choices=sorted(ROUTES))
    parser.add_argument("--timeout", type=float, default=900.0, help="wall-clock seconds")
    args = parser.parse_args()

    samples, failure = _record(args.route, args.timeout)
    if failure or len(samples) < 2:
        print(f"odom_drift: FAILED: {failure or 'no samples recorded'}")
        sys.exit(1)

    t0 = samples[0][0]
    times = [s[0] - t0 for s in samples]
    series = {}
    for i, key in enumerate(("truth", "odom", "ekf"), start=1):
        origin = samples[0][i]
        series[key] = [relative_to(origin, s[i]) for s in samples]
    truth, odom, ekf = series["truth"], series["odom"], series["ekf"]
    stats_odom = position_errors(odom, truth)
    stats_ekf = position_errors(ekf, truth)
    err_odom = [math.hypot(o.x - g.x, o.y - g.y) for o, g in zip(odom, truth)]
    err_ekf = [math.hypot(e.x - g.x, e.y - g.y) for e, g in zip(ekf, truth)]
    driven = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(truth, truth[1:]))

    RESULTS.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = RESULTS / f"odom_drift_{stamp}"
    with open(f"{base}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "truth_x", "truth_y", "truth_yaw", "odom_x", "odom_y", "odom_yaw",
                    "ekf_x", "ekf_y", "ekf_yaw", "odom_err_m", "ekf_err_m"])
        for t, g, o, e, eo, ee in zip(times, truth, odom, ekf, err_odom, err_ekf):
            w.writerow([f"{t:.2f}", f"{g.x:.4f}", f"{g.y:.4f}", f"{g.yaw:.4f}",
                        f"{o.x:.4f}", f"{o.y:.4f}", f"{o.yaw:.4f}",
                        f"{e.x:.4f}", f"{e.y:.4f}", f"{e.yaw:.4f}", f"{eo:.4f}", f"{ee:.4f}"])
    _plot(f"{base}.png", truth, odom, ekf, times, err_odom, err_ekf)

    rows = [("Raw wheel odometry (`odom`)", stats_odom, _heading_error_deg(odom[-1], truth[-1])),
            ("EKF (`odometry/filtered`)", stats_ekf, _heading_error_deg(ekf[-1], truth[-1]))]
    table = ["| Estimate | mean error (m) | max error (m) | final error (m) "
             "| final heading error (deg) |",
             "|---|---|---|---|---|"]
    table += [f"| {name} | {s.mean:.3f} | {s.max:.3f} | {s.final:.3f} | {h:+.2f} |"
              for name, s, h in rows]
    summary = "\n".join([
        f"# Odometry drift, {stamp}",
        "",
        f"- Route: `{args.route}`, planned {route_length(ROUTES[args.route], (0.0, 0.0)):.1f} m;"
        f" driven {driven:.1f} m (ground truth) in {times[-1]:.1f} s of sim time.",
        f"- Odometry error model: {args.label}.",
        f"- Samples: {len(samples)} at 10 Hz of sim time. Error = xy distance from ground truth,"
        " each path relative to its own start pose.",
        f"- Code: commit `{_git_commit()}`, `benchmarks/odom_drift.py`.",
        "",
        *table,
        "",
        f"Raw data: `{base.name}.csv`. Plot: `{base.name}.png`.",
        "",
    ])
    Path(f"{base}.md").write_text(summary)
    print(summary)


if __name__ == "__main__":
    main()
