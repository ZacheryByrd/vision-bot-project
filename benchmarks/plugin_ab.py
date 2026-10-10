#!/usr/bin/env python3
"""RobotAvoidanceLayer A/B benchmark (Gate 4).

Needs nav2.launch.py running (the world must have the gazebo_ros_state
plugin, as nav_world.world does). Spawns a second robot, "robot2": a box
with this robot's footprint (0.20 x 0.19 x 0.066 m, below the 0.083 m lidar
plane, so the lidar cannot see it, exactly like a real second vision_bot),
moved by a planar_move plugin. Its true pose is republished as
/robot2/amcl_pose at 10 Hz, standing in for its own localization.

Each trial puts both robots at the scenario's start poses (teleport, AMCL
re-initialized, costmaps cleared), sends one NavigateToPose goal, and records
both robots' ground truth. Conditions: the layer disabled vs enabled, set at
runtime on both costmaps. Every (scenario, seed) runs once per condition with
identical robot2 placement and timing; which condition runs first alternates.

Scenarios:
    parked_doorway  robot2 stands in the east doorway (1.5, -0.3 +- 0.1, yaw
                    +-0.3); the robot drives (0, -0.3) -> (3.0, -0.3) through it.
    crossing        robot2 drives south at 0.15 m/s from (0.3, 0.6) after a
                    0-6 s delay; the robot drives (-2.0, -0.3) -> (2.4, -0.3).

Metrics per condition: summarize_goals plus the minimum centre-to-centre
distance between the robots and the number of contacts (< CONTACT_M).
Writes benchmarks/results/plugin_ab_<timestamp>.{csv,md}.
    python3 benchmarks/plugin_ab.py --per-scenario 10
"""

import argparse
import bisect
import csv
import math
import random
import sys
import time
from dataclasses import dataclass
from pathlib import Path

from metrics import GoalResult, summarize_goals
from provenance import git_commit

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"

# Centres closer than this mean the bodies touched: head-on, two 0.20 m long
# robots touch at 0.20 m; side by side (0.19 m wide) at 0.19 m.
CONTACT_M = 0.20

# A goal counts as truly reached when Nav2 reports success AND ground truth
# puts the robot within this distance of it (0.15 m goal tolerance plus
# localization error). A collision can shove the robot and corrupt AMCL, so
# Nav2 may report success while the robot is somewhere else.
ARRIVED_M = 0.30


@dataclass(frozen=True)
class ScenarioSpec:
    start: tuple            # robot (x, y, yaw)
    goal: tuple             # robot goal (x, y, yaw)
    timeout_s: float        # sim seconds
    robot2_speed: float     # m/s forward while driving (0 = parked)
    robot2_drive_s: float   # how long robot2 drives after its delay


SCENARIOS = {
    "parked_doorway": ScenarioSpec((0.0, -0.3, 0.0), (3.0, -0.3, 0.0), 90.0, 0.0, 0.0),
    "crossing": ScenarioSpec((-2.0, -0.3, 0.0), (2.4, -0.3, 0.0), 75.0, 0.15, 25.0),
}


@dataclass(frozen=True)
class Trial:
    scenario: str
    seed: int
    layer_enabled: bool
    robot2_start: tuple     # (x, y, yaw)
    robot2_delay_s: float


@dataclass(frozen=True)
class TrialRow:
    scenario: str
    seed: int
    layer_enabled: bool
    result: GoalResult
    min_separation_m: float   # None if the tracks never overlapped in time
    status: str


@dataclass(frozen=True)
class ABSummary:
    goals: object             # metrics.GoalSummary
    mean_separation_m: float
    worst_separation_m: float
    contacts: int
    arrivals: int             # success and final_error_m <= ARRIVED_M


def _robot2_setup(scenario, seed):
    rng = random.Random(f"{scenario}-{seed}")
    if scenario == "parked_doorway":
        return (1.5, -0.3 + rng.uniform(-0.1, 0.1), rng.uniform(-0.3, 0.3)), 0.0
    if scenario == "crossing":
        return (0.3, 0.6, -math.pi / 2), rng.uniform(0.0, 6.0)
    raise ValueError(f"unknown scenario {scenario}")


def make_trials(per_scenario, base_seed):
    """Every (scenario, seed) once per condition, with identical robot2 setup."""
    trials = []
    for k in range(per_scenario):
        seed = base_seed + k
        for i, name in enumerate(SCENARIOS):
            start, delay = _robot2_setup(name, seed)
            order = (False, True) if (k + i) % 2 == 0 else (True, False)
            for enabled in order:
                trials.append(Trial(name, seed, enabled, start, delay))
    return trials


def min_separation(track_a, track_b, max_dt=0.1):
    """Smallest distance between (t, x, y) tracks at matching times, or None."""
    times_b = [s[0] for s in track_b]
    best = None
    for t, x, y in track_a:
        k = bisect.bisect_left(times_b, t)
        for j in (k - 1, k):
            if 0 <= j < len(track_b) and abs(track_b[j][0] - t) <= max_dt:
                d = math.hypot(track_b[j][1] - x, track_b[j][2] - y)
                best = d if best is None else min(best, d)
    return best


def summarize_ab(rows):
    """ABSummary per (scenario, layer_enabled)."""
    groups = {}
    for r in rows:
        groups.setdefault((r.scenario, r.layer_enabled), []).append(r)
    out = {}
    for key, group in groups.items():
        seps = [r.min_separation_m for r in group if r.min_separation_m is not None]
        out[key] = ABSummary(
            goals=summarize_goals([r.result for r in group]),
            mean_separation_m=sum(seps) / len(seps) if seps else float("nan"),
            worst_separation_m=min(seps) if seps else float("nan"),
            contacts=sum(1 for s in seps if s < CONTACT_M),
            arrivals=sum(1 for r in group
                         if r.result.success and r.result.final_error_m <= ARRIVED_M),
        )
    return out


ROBOT2_SDF = """<sdf version='1.6'><model name='robot2'>
<link name='base_link'>
  <inertial><mass>1.45</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
  <collision name='body'><geometry><box><size>0.20 0.19 0.066</size></box></geometry>
    <surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision>
  <visual name='body'><geometry><box><size>0.20 0.19 0.066</size></box></geometry>
    <material><ambient>1 0.5 0 1</ambient><diffuse>1 0.5 0 1</diffuse></material></visual>
</link>
<plugin name='planar_move' filename='libgazebo_ros_planar_move.so'>
  <ros><namespace>/robot2</namespace></ros>
  <update_rate>50</update_rate><publish_rate>20</publish_rate>
  <publish_odom>true</publish_odom><publish_odom_tf>false</publish_odom_tf>
  <odometry_frame>map</odometry_frame><robot_base_frame>robot2/base_link</robot_base_frame>
</plugin></model></sdf>"""


def _run(trials):
    import rclpy
    from action_msgs.msg import GoalStatus
    from gazebo_msgs.msg import EntityState
    from gazebo_msgs.srv import SetEntityState, SpawnEntity
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import ClearEntireCostmap
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data

    from drive_route import make_node

    rclpy.init()
    node = make_node("plugin_ab")
    st = {"r2": None, "truth": None, "rec": False, "t1": [], "t2": [], "recoveries": 0}

    def now():
        return node.get_clock().now().nanoseconds * 1e-9

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def on_truth(msg):
        p = msg.pose.pose.position
        st["truth"] = (p.x, p.y)
        if st["rec"]:
            st["t1"].append((stamp(msg), p.x, p.y))

    def on_r2(msg):
        st["r2"] = msg
        if st["rec"]:
            p = msg.pose.pose.position
            st["t2"].append((stamp(msg), p.x, p.y))

    node.create_subscription(Odometry, "ground_truth/pose", on_truth, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/robot2/odom", on_r2, 10)
    relay = node.create_publisher(PoseWithCovarianceStamped, "/robot2/amcl_pose", 10)
    r2_cmd = node.create_publisher(Twist, "/robot2/cmd_vel", 10)
    init_pose = node.create_publisher(PoseWithCovarianceStamped, "initialpose", 10)
    r2_speed = {"v": 0.0}

    def tick():
        if st["r2"] is not None:
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose = st["r2"].pose.pose
            relay.publish(msg)
        cmd = Twist()
        cmd.linear.x = r2_speed["v"]
        r2_cmd.publish(cmd)

    node.create_timer(0.1, tick)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def call(client, request, wall_s=30.0):
        if not client.wait_for_service(timeout_sec=wall_s):
            raise SystemExit(f"service {client.srv_name} not available")
        future = client.call_async(request)
        end = time.monotonic() + wall_s
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        return future.result()

    spawn = node.create_client(SpawnEntity, "/spawn_entity")
    set_state = node.create_client(SetEntityState, "/gazebo/set_entity_state")
    clears = [node.create_client(ClearEntireCostmap, f"/{c}/clear_entirely_{c}")
              for c in ("global_costmap", "local_costmap")]
    params = [node.create_client(SetParameters, f"/{c}/{c}/set_parameters")
              for c in ("global_costmap", "local_costmap")]
    nav = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not nav.wait_for_server(timeout_sec=120.0):
        raise SystemExit("navigate_to_pose not available")
    spin(1.0)
    call(spawn, SpawnEntity.Request(name="robot2", xml=ROBOT2_SDF))

    def teleport(name, x, y, yaw):
        state = EntityState(name=name, reference_frame="world")
        state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, 0.035
        state.pose.orientation.z, state.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        call(set_state, SetEntityState.Request(state=state))

    def set_layer(enabled):
        value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=enabled)
        for client in params:
            call(client, SetParameters.Request(
                parameters=[Parameter(name="robot_avoidance_layer.enabled", value=value)]))

    def on_feedback(fb):
        st["recoveries"] = max(st["recoveries"], fb.feedback.number_of_recoveries)

    rows, layer_state = [], None
    for n, trial in enumerate(trials, start=1):
        spec = SCENARIOS[trial.scenario]
        if trial.layer_enabled != layer_state:
            set_layer(trial.layer_enabled)
            layer_state = trial.layer_enabled
        r2_speed["v"] = 0.0
        teleport("vision_bot", *spec.start)
        teleport("robot2", *trial.robot2_start)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = "map"
        initial.pose.pose.position.x, initial.pose.pose.position.y = spec.start[:2]
        initial.pose.pose.orientation.z = math.sin(spec.start[2] / 2)
        initial.pose.pose.orientation.w = math.cos(spec.start[2] / 2)
        initial.pose.covariance[0] = initial.pose.covariance[7] = 0.01
        initial.pose.covariance[35] = 0.01
        for _ in range(3):
            initial.header.stamp = node.get_clock().now().to_msg()
            init_pose.publish(initial)
            spin(0.5)
        spin(1.5)
        for client in clears:
            call(client, ClearEntireCostmap.Request())
        spin(2.0)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = spec.goal[:2]
        goal.pose.pose.orientation.z = math.sin(spec.goal[2] / 2)
        goal.pose.pose.orientation.w = math.cos(spec.goal[2] / 2)
        st.update(rec=True, t1=[], t2=[], recoveries=0)
        send = nav.send_goal_async(goal, feedback_callback=on_feedback)
        end = time.monotonic() + 30.0
        while not send.done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        handle = send.result() if send.done() else None
        t0, status = now(), "rejected"
        if handle is not None and handle.accepted:
            result = handle.get_result_async()
            while not result.done() and now() - t0 < spec.timeout_s:
                since_delay = now() - t0 - trial.robot2_delay_s
                driving = 0.0 <= since_delay < spec.robot2_drive_s
                r2_speed["v"] = spec.robot2_speed if driving else 0.0
                rclpy.spin_once(node, timeout_sec=0.05)
            if result.done():
                code = result.result().status
                status = {GoalStatus.STATUS_SUCCEEDED: "succeeded",
                          GoalStatus.STATUS_ABORTED: "aborted",
                          GoalStatus.STATUS_CANCELED: "canceled"}.get(code, str(code))
            else:
                status = "timeout"
                cancel = handle.cancel_goal_async()
                end = time.monotonic() + 15.0
                while not cancel.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
        elapsed = now() - t0
        r2_speed["v"] = 0.0
        spin(1.0)
        st["rec"] = False
        track = [(x, y) for _, x, y in st["t1"]]
        result_row = GoalResult(
            goal_id=f"{trial.scenario}-{trial.seed}", success=status == "succeeded",
            time_s=elapsed,
            path_length_m=sum(math.dist(a, b) for a, b in zip(track, track[1:])),
            final_error_m=math.dist(st["truth"], spec.goal[:2]),
            recoveries=st["recoveries"])
        row = TrialRow(trial.scenario, trial.seed, trial.layer_enabled, result_row,
                       min_separation(st["t1"], st["t2"]), status)
        rows.append(row)
        sep = "n/a" if row.min_separation_m is None else f"{row.min_separation_m:.3f} m"
        print(f"[{n}/{len(trials)}] {trial.scenario:<15} seed {trial.seed} "
              f"layer {'on ' if trial.layer_enabled else 'off'}  {status:<9} "
              f"{elapsed:5.1f} s  min separation {sep}  recoveries {row.result.recoveries}",
              flush=True)

    node.destroy_node()
    rclpy.shutdown()
    return rows


def _fmt(value, spec):
    return "n/a" if value is None or (isinstance(value, float) and math.isnan(value)) \
        else format(value, spec)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--per-scenario", type=int, default=10,
                        help="trials per scenario per condition")
    parser.add_argument("--base-seed", type=int, default=0)
    parser.add_argument("--label", default="RobotAvoidanceLayer as in config/nav2_params.yaml")
    args = parser.parse_args()

    trials = make_trials(args.per_scenario, args.base_seed)
    rows = _run(trials)
    trial_by_key = {(t.scenario, t.seed, t.layer_enabled): t for t in trials}
    summary = summarize_ab(rows)

    RESULTS.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = RESULTS / f"plugin_ab_{stamp}"
    with open(f"{base}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "seed", "layer_enabled", "robot2_x", "robot2_y", "robot2_yaw",
                    "robot2_delay_s", "status", "success", "time_s", "path_length_m",
                    "final_error_m", "recoveries", "min_separation_m", "contact"])
        for r in rows:
            t = trial_by_key[(r.scenario, r.seed, r.layer_enabled)]
            sep = r.min_separation_m
            w.writerow([r.scenario, r.seed, r.layer_enabled, *[f"{v:.3f}" for v in t.robot2_start],
                        f"{t.robot2_delay_s:.2f}", r.status, r.result.success,
                        f"{r.result.time_s:.2f}", f"{r.result.path_length_m:.3f}",
                        f"{r.result.final_error_m:.3f}", r.result.recoveries,
                        "" if sep is None else f"{sep:.3f}",
                        sep is not None and sep < CONTACT_M])

    table = ["| Scenario | Layer | Nav2 success | Truly arrived | Mean time (s) "
             "| Mean final error (m) | Recoveries "
             "| Mean min separation (m) | Worst min separation (m) | Contacts |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for name in SCENARIOS:
        for enabled in (False, True):
            s = summary.get((name, enabled))
            if s is None:
                continue
            ok = round(s.goals.success_rate * s.goals.n)
            table.append(
                f"| {name} | {'on' if enabled else 'off'} | {ok}/{s.goals.n} "
                f"| {s.arrivals}/{s.goals.n} "
                f"| {_fmt(s.goals.mean_time_s, '.1f')} | {_fmt(s.goals.mean_final_error_m, '.3f')} "
                f"| {s.goals.total_recoveries} | {_fmt(s.mean_separation_m, '.3f')} "
                f"| {_fmt(s.worst_separation_m, '.3f')} | {s.contacts}/{s.goals.n} |")
    md = "\n".join([
        f"# RobotAvoidanceLayer A/B, {stamp}",
        "",
        f"- Configuration: {args.label}.",
        f"- {args.per_scenario} trials per scenario per condition, seeds {args.base_seed} to "
        f"{args.base_seed + args.per_scenario - 1}; identical robot2 placement and timing in "
        "both conditions; condition order alternates.",
        f"- Contact: robot centres closer than {CONTACT_M} m (the bodies touched). "
        f"Truly arrived: Nav2 success and the robot within {ARRIVED_M} m of the goal by ground "
        "truth. Means of time and final error are over Nav2 successes.",
        f"- Code: commit `{git_commit()}`, `benchmarks/plugin_ab.py`.",
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
