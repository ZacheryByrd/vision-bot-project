#!/usr/bin/env python3
"""Nav2 goal benchmark (Gate 3).

Needs nav2.launch.py running. Sends each goal in benchmarks/goals.yaml, in
order, through the NavigateToPose action with a per-goal timeout, and
scores it against ground truth:
    success         the action finished SUCCEEDED (Nav2's own judgement,
                    made from its estimated pose)
    time_s          sim time from goal acceptance to result
    path_length_m   distance the robot actually travelled (ground truth)
    final_error_m   ground-truth distance from the robot to the goal point
    recoveries      Nav2's number_of_recoveries feedback for the goal
The CSV also records AMCL's own position error when each goal ends.

Writes benchmarks/results/nav_goals_<timestamp>.{csv,md}.
    python3 benchmarks/nav_goals.py --label "baseline: installed defaults"
"""

import argparse
import csv
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import yaml

from metrics import GoalResult, summarize_goals
from provenance import git_commit

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"


@dataclass(frozen=True)
class Goal:
    goal_id: str
    x: float
    y: float
    yaw: float


def load_goals(text):
    """Goals from a goals.yaml document; ids must be unique and non-empty."""
    goals = [Goal(str(g["id"]), float(g["x"]), float(g["y"]), float(g["yaw"]))
             for g in (yaml.safe_load(text) or {}).get("goals") or []]
    if not goals:
        raise ValueError("no goals defined")
    ids = [g.goal_id for g in goals]
    if len(ids) != len(set(ids)):
        raise ValueError(f"duplicate goal ids in {ids}")
    return goals


def path_length(points):
    """Length of the polyline through (x, y) points."""
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def _run(goals, timeout_s):
    """Drive every goal; returns a list of (GoalResult, extras dict)."""
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data

    from drive_route import make_node

    rclpy.init()
    node = make_node("nav_goals")
    state = {"truth": None, "amcl": None, "track": None, "recoveries": 0}

    def on_truth(msg):
        p = msg.pose.pose.position
        state["truth"] = (p.x, p.y)
        if state["track"] is not None:
            state["track"].append((p.x, p.y))

    def on_amcl(msg):
        p = msg.pose.pose.position
        state["amcl"] = (p.x, p.y)

    def on_feedback(fb):
        state["recoveries"] = max(state["recoveries"], fb.feedback.number_of_recoveries)

    node.create_subscription(Odometry, "ground_truth/pose", on_truth, qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, "amcl_pose", on_amcl, 10)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def spin_until(done, wall_s):
        end = time.monotonic() + wall_s
        while not done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        return done()

    if not client.wait_for_server(timeout_sec=120.0):
        raise SystemExit("navigate_to_pose action server not available")
    spin_until(lambda: state["truth"] is not None and now() > 0.0, 30.0)

    results = []
    for goal in goals:
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = "map"
        msg.pose.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = goal.x, goal.y
        msg.pose.pose.orientation.z = math.sin(goal.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(goal.yaw / 2)
        state["track"], state["recoveries"] = [state["truth"]], 0

        send = client.send_goal_async(msg, feedback_callback=on_feedback)
        spin_until(send.done, 30.0)
        handle = send.result() if send.done() else None
        status_name = "rejected"
        t0 = now()
        success = False
        if handle is not None and handle.accepted:
            result = handle.get_result_async()
            while not result.done() and now() - t0 < timeout_s:
                rclpy.spin_once(node, timeout_sec=0.1)
            if result.done():
                status = result.result().status
                success = status == GoalStatus.STATUS_SUCCEEDED
                status_name = {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                               GoalStatus.STATUS_ABORTED: "aborted",
                               GoalStatus.STATUS_CANCELED: "canceled"}.get(status, str(status))
            else:
                status_name = "timeout"
                cancel = handle.cancel_goal_async()
                spin_until(cancel.done, 15.0)
        elapsed = now() - t0
        spin_until(lambda: False, 1.0)   # let the last truth/AMCL messages arrive
        truth, amcl = state["truth"], state["amcl"]
        res = GoalResult(goal_id=goal.goal_id, success=success, time_s=elapsed,
                         path_length_m=path_length(state["track"]),
                         final_error_m=math.dist(truth, (goal.x, goal.y)),
                         recoveries=state["recoveries"])
        extras = {"status": status_name,
                  "amcl_error_m": math.dist(amcl, truth) if amcl else float("nan")}
        state["track"] = None
        results.append((res, extras))
        print(f"{goal.goal_id:<20} {status_name:<10} {elapsed:6.1f} s  "
              f"path {res.path_length_m:5.2f} m  error {res.final_error_m:.3f} m  "
              f"AMCL error {extras['amcl_error_m']:.3f} m  recoveries {res.recoveries}",
              flush=True)

    node.destroy_node()
    rclpy.shutdown()
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--label", required=True, help="configuration under test")
    parser.add_argument("--goals", default=str(HERE / "goals.yaml"))
    parser.add_argument("--timeout", type=float, default=180.0, help="per goal, sim seconds")
    args = parser.parse_args()

    goals = load_goals(Path(args.goals).read_text())
    rows = _run(goals, args.timeout)
    summary = summarize_goals([r for r, _ in rows])

    RESULTS.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = RESULTS / f"nav_goals_{stamp}"
    with open(f"{base}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["goal_id", "x", "y", "yaw", "success", "status", "time_s",
                    "path_length_m", "final_error_m", "amcl_error_m", "recoveries"])
        for goal, (r, extra) in zip(goals, rows):
            w.writerow([r.goal_id, goal.x, goal.y, goal.yaw, r.success, extra["status"],
                        f"{r.time_s:.2f}", f"{r.path_length_m:.3f}", f"{r.final_error_m:.3f}",
                        f"{extra['amcl_error_m']:.3f}", r.recoveries])
    table = ["| Goal | Result | Time (s) | Path (m) | Final error (m) | AMCL error (m) "
             "| Recoveries |", "|---|---|---|---|---|---|---|"]
    table += [f"| {r.goal_id} | {extra['status']} | {r.time_s:.1f} | {r.path_length_m:.2f} "
              f"| {r.final_error_m:.3f} | {extra['amcl_error_m']:.3f} | {r.recoveries} |"
              for r, extra in rows]
    succeeded = round(summary.success_rate * summary.n)
    md = "\n".join([
        f"# Nav2 goal benchmark, {stamp}",
        "",
        f"- Configuration: {args.label}.",
        f"- Goals: `{Path(args.goals).name}`, {summary.n} goals driven in order; "
        f"per-goal timeout {args.timeout:.0f} s of sim time.",
        f"- Code: commit `{git_commit()}`, `benchmarks/nav_goals.py`.",
        "",
        f"**Success: {succeeded} of {summary.n} ({summary.success_rate:.0%}).** Over successful "
        f"goals: mean time {summary.mean_time_s:.1f} s, mean final error "
        f"{summary.mean_final_error_m:.3f} m (ground truth). Recoveries: "
        f"{summary.total_recoveries}.",
        "",
        "Final error is the ground-truth distance from the robot to the goal point when Nav2 "
        "finished; it includes AMCL's localization error and the goal tolerance.",
        "",
        *table,
        "",
        f"Raw data: `{base.name}.csv`.",
        "",
    ])
    Path(f"{base}.md").write_text(md)
    print(md)
    sys.exit(0)


if __name__ == "__main__":
    main()
