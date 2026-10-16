#!/usr/bin/env python3
"""EKF regression check: replay a recorded drive through the repo's EKF config.

Runs robot_localization's ekf_node with ros2_ws/src/vision_bot_nav/config/
ekf.yaml against `ros2 bag play` of a bag recorded by
record_regression_bag.sh (sim clock, wheel odometry, IMU, lidar, static
transforms, ground truth), then scores the EKF output against the bag's
ground truth the same way odom_drift.py does (each path relative to its own
start pose, metrics.position_errors). Exits 0 if the mean position error is
at or below the threshold, 1 if it is above, 2 if the replay produced no
data. No Gazebo is needed, so it runs in CI.

    python3 benchmarks/replay_regression.py --bag benchmarks/bags/drift_loop
    python3 benchmarks/replay_regression.py --bag benchmarks/bags/drift_loop --threshold-m 0.05

The default threshold is threshold_m in benchmarks/baseline.json.
"""

import argparse
import bisect
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from metrics import position_errors, relative_to

HERE = Path(__file__).resolve().parent
EKF_CONFIG = HERE.parent / "ros2_ws" / "src" / "vision_bot_nav" / "config" / "ekf.yaml"
BASELINE = HERE / "baseline.json"


def align(estimates, truths, max_dt):
    """(estimate, truth) pairs: each (t, value) estimate with the truth nearest in time, if
    within max_dt seconds. Both lists sorted by time."""
    times = [t for t, _ in truths]
    pairs = []
    for t, value in estimates:
        i = bisect.bisect_left(times, t)
        candidates = [j for j in (i - 1, i) if 0 <= j < len(times)]
        if not candidates:
            continue
        j = min(candidates, key=lambda k: abs(times[k] - t))
        if abs(times[j] - t) <= max_dt:
            pairs.append((value, truths[j][1]))
    return pairs


def score(pairs):
    """ErrorStats of the estimate against the truth, each relative to its own first pose."""
    if not pairs:
        raise ValueError("no aligned samples")
    est0, truth0 = pairs[0]
    return position_errors([relative_to(est0, e) for e, _ in pairs],
                           [relative_to(truth0, t) for _, t in pairs])


def passes(mean_m, threshold_m):
    return mean_m <= threshold_m


def read_threshold(path):
    return float(json.loads(Path(path).read_text())["threshold_m"])


def replay(bag, ekf_config, rate):
    """Play the bag through a fresh ekf_node; returns (ekf samples, truth samples) as
    sorted lists of (sim time s, Pose2D)."""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data

    from route import pose_from_odometry

    rclpy.init()
    node = rclpy.create_node("replay_regression", parameter_overrides=[
        Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    ekf, truth = [], []

    def keep(store):
        def cb(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            store.append((stamp, pose_from_odometry(msg)))
        return cb

    node.create_subscription(Odometry, "/odometry/filtered", keep(ekf), qos_profile_sensor_data)
    node.create_subscription(Odometry, "/ground_truth/pose", keep(truth), qos_profile_sensor_data)

    env = dict(os.environ)
    ekf_proc = subprocess.Popen(
        ["ros2", "run", "robot_localization", "ekf_node", "--ros-args", "-r",
         "__node:=ekf_filter_node", "--params-file", str(ekf_config),
         "-p", "use_sim_time:=true"],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        end = time.monotonic() + 5.0            # let the EKF and this node discover each other
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        play = subprocess.Popen(["ros2", "bag", "play", str(bag), "--rate", str(rate)],
                                env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        while play.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.05)
        end = time.monotonic() + 1.0
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        ekf_proc.terminate()
        ekf_proc.wait(timeout=10)
        node.destroy_node()
        rclpy.shutdown()
    return sorted(ekf), sorted(truth)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--threshold-m", type=float,
                        help="pass if the mean error is at or below this (default: baseline.json)")
    parser.add_argument("--ekf-config", type=Path, default=EKF_CONFIG)
    parser.add_argument("--rate", type=float, default=1.0, help="bag playback rate")
    parser.add_argument("--json-out", type=Path, help="also write the result as JSON")
    args = parser.parse_args()
    threshold = args.threshold_m if args.threshold_m is not None else read_threshold(BASELINE)

    ekf, truth = replay(args.bag, args.ekf_config, args.rate)
    pairs = align(ekf, truth, max_dt=0.02)
    if len(pairs) < 100:
        print(f"replay_regression: FAILED: only {len(pairs)} aligned samples "
              f"({len(ekf)} EKF, {len(truth)} ground truth)")
        sys.exit(2)
    stats = score(pairs)
    ok = passes(stats.mean, threshold)
    print(f"EKF vs ground truth over {len(pairs)} samples: mean {stats.mean:.4f} m, "
          f"max {stats.max:.4f} m, final {stats.final:.4f} m; threshold {threshold:.4f} m: "
          f"{'PASS' if ok else 'FAIL'}")
    if args.json_out:
        args.json_out.write_text(json.dumps({
            "bag": str(args.bag), "samples": len(pairs), "mean_m": stats.mean,
            "max_m": stats.max, "final_m": stats.final, "threshold_m": threshold,
            "passed": ok}, indent=2) + "\n")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
