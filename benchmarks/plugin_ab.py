#!/usr/bin/env python3
"""RobotAvoidanceLayer A/B benchmark (Gate 4).

Needs nav2.launch.py running (the world must have the gazebo_ros_state
plugin, as nav_world.world does). Spawns a second robot, "robot2": a box
with this robot's footprint (0.20 x 0.19 x 0.066 m, below the 0.083 m lidar
plane, so the lidar cannot see it, exactly like a real second vision_bot),
moved by a planar_move plugin. Its true pose is republished as
/robot2/shared_pose at 10 Hz, standing in for its own localization.

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

Two-robot mode (--robot1 robot1 --robot2 robot2, with multi_robot.launch.py
running): robot2 is a second full vision_bot with its own Nav2, localization
and RobotAvoidanceLayer, so nothing is spawned or relayed. Both robots are
teleported and re-initialized each trial, the layer is switched on all four
costmaps, and in the crossing scenario robot2 gets its own Nav2 goal south
across the route after the seeded delay instead of a scripted drive (it is
cancelled when the trial ends). In parked_doorway robot2 gets no goal.

Metrics per condition: summarize_goals plus the minimum centre-to-centre
distance between the robots and the number of contacts (< CONTACT_M).
Writes benchmarks/results/plugin_ab_[2robots_]<timestamp>.{csv,md}.
    python3 benchmarks/plugin_ab.py --per-scenario 10
    python3 benchmarks/plugin_ab.py --per-scenario 10 --robot1 robot1 --robot2 robot2
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
from nav_goals import ns_topic
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
    robot2_goal: tuple = None   # two-robot mode: robot2's Nav2 goal after its delay


SCENARIOS = {
    "parked_doorway": ScenarioSpec((0.0, -0.3, 0.0), (3.0, -0.3, 0.0), 90.0, 0.0, 0.0),
    "crossing": ScenarioSpec((-2.0, -0.3, 0.0), (2.4, -0.3, 0.0), 75.0, 0.15, 25.0,
                             (0.3, -2.0, -math.pi / 2)),
}

COSTMAPS = ("global_costmap", "local_costmap")


@dataclass(frozen=True)
class RobotIO:
    """Gazebo entity and ROS names of one robot's Nav2 stack."""
    entity: str
    truth: str
    initialpose: str
    navigate: str
    clear: tuple            # ClearEntireCostmap services
    params: tuple           # costmap SetParameters services


def robot_io(namespace):
    """RobotIO for multi_robot.launch.py's robot <namespace>, or nav2.launch.py's if ""."""
    return RobotIO(
        entity=namespace.strip("/") or "vision_bot",
        truth=ns_topic(namespace, "ground_truth/pose"),
        initialpose=ns_topic(namespace, "initialpose"),
        navigate=ns_topic(namespace, "navigate_to_pose"),
        clear=tuple(ns_topic(namespace, f"{c}/clear_entirely_{c}") for c in COSTMAPS),
        params=tuple(ns_topic(namespace, f"{c}/{c}/set_parameters") for c in COSTMAPS),
    )


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
    robot2_status: str = ""   # two-robot mode: outcome of robot2's own goal


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


def robot2_arrivals(rows, scenario, layer_enabled):
    """(trials where robot2's own goal succeeded, trials) for one scenario and condition."""
    group = [r for r in rows if r.scenario == scenario and r.layer_enabled == layer_enabled]
    return sum(1 for r in group if r.robot2_status == "succeeded"), len(group)


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


def _run(trials, robot1="", robot2=""):
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
    me = robot_io(robot1)
    real = bool(robot2)          # robot2 is a full vision_bot with its own Nav2
    other = robot_io(robot2) if real else None
    names = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
             GoalStatus.STATUS_CANCELED: "canceled"}

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

    node.create_subscription(Odometry, me.truth, on_truth, qos_profile_sensor_data)
    init_pose = node.create_publisher(PoseWithCovarianceStamped, me.initialpose, 10)
    r2_speed = {"v": 0.0}
    if real:
        node.create_subscription(Odometry, other.truth, on_r2, qos_profile_sensor_data)
        init_pose2 = node.create_publisher(PoseWithCovarianceStamped, other.initialpose, 10)
    else:
        node.create_subscription(Odometry, "/robot2/odom", on_r2, 10)
        relay = node.create_publisher(PoseWithCovarianceStamped, "/robot2/shared_pose", 10)
        r2_cmd = node.create_publisher(Twist, "/robot2/cmd_vel", 10)

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

    def spin_until(done, wall_s):
        end = time.monotonic() + wall_s
        while not done() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        return done()

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
    stacks = (me, other) if real else (me,)
    clears = [node.create_client(ClearEntireCostmap, s) for io in stacks for s in io.clear]
    params = [node.create_client(SetParameters, s) for io in stacks for s in io.params]
    navs = [ActionClient(node, NavigateToPose, io.navigate) for io in stacks]
    for client, io in zip(navs, stacks):
        if not client.wait_for_server(timeout_sec=120.0):
            raise SystemExit(f"{io.navigate} not available")
    nav, nav2 = navs[0], navs[-1] if real else None
    spin(1.0)
    if not real:
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

    def pose_msg(msg, x, y, yaw):
        msg.header.frame_id = "map"
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        return msg

    def nav_goal(x, y, yaw):
        goal = NavigateToPose.Goal()
        pose_msg(goal.pose, x, y, yaw)
        return goal

    def finish_robot2(r2):
        """Outcome of robot2's goal at the end of a trial; cancels it if still running."""
        if r2["send"] is None:
            return "no goal"
        spin_until(r2["send"].done, 10.0)
        handle = r2["send"].result() if r2["send"].done() else None
        if handle is None or not handle.accepted:
            return "rejected"
        if r2["result"] is None:
            r2["result"] = handle.get_result_async()
        if spin_until(r2["result"].done, 1.0):
            code = r2["result"].result().status
            return names.get(code, str(code))
        cancel = handle.cancel_goal_async()
        spin_until(cancel.done, 15.0)
        return "cancelled at trial end"

    rows, layer_state = [], None
    for n, trial in enumerate(trials, start=1):
        spec = SCENARIOS[trial.scenario]
        if trial.layer_enabled != layer_state:
            set_layer(trial.layer_enabled)
            layer_state = trial.layer_enabled
        r2_speed["v"] = 0.0
        teleport(me.entity, *spec.start)
        teleport(other.entity if real else "robot2", *trial.robot2_start)
        initials = [(init_pose, spec.start)]
        if real:
            initials.append((init_pose2, trial.robot2_start))
        for _ in range(3):
            for publisher, pose in initials:
                initial = pose_msg(PoseWithCovarianceStamped(), *pose)
                initial.pose.covariance[0] = initial.pose.covariance[7] = 0.01
                initial.pose.covariance[35] = 0.01
                publisher.publish(initial)
            spin(0.5)
        spin(1.5)
        for client in clears:
            call(client, ClearEntireCostmap.Request())
        spin(2.0)

        st.update(rec=True, t1=[], t2=[], recoveries=0)
        r2 = {"send": None, "result": None}     # robot2's own goal (two-robot mode)
        send = nav.send_goal_async(nav_goal(*spec.goal), feedback_callback=on_feedback)
        spin_until(send.done, 30.0)
        handle = send.result() if send.done() else None
        t0, status = now(), "rejected"
        if handle is not None and handle.accepted:
            result = handle.get_result_async()
            while not result.done() and now() - t0 < spec.timeout_s:
                since_delay = now() - t0 - trial.robot2_delay_s
                if not real:
                    driving = 0.0 <= since_delay < spec.robot2_drive_s
                    r2_speed["v"] = spec.robot2_speed if driving else 0.0
                elif spec.robot2_goal is not None and since_delay >= 0.0 and r2["send"] is None:
                    r2["send"] = nav2.send_goal_async(nav_goal(*spec.robot2_goal))
                elif r2["send"] is not None and r2["result"] is None and r2["send"].done() \
                        and r2["send"].result().accepted:
                    r2["result"] = r2["send"].result().get_result_async()
                rclpy.spin_once(node, timeout_sec=0.05)
            if result.done():
                code = result.result().status
                status = names.get(code, str(code))
            else:
                status = "timeout"
                cancel = handle.cancel_goal_async()
                spin_until(cancel.done, 15.0)
        elapsed = now() - t0
        r2_speed["v"] = 0.0
        robot2_status = finish_robot2(r2) if real else "scripted"
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
                       min_separation(st["t1"], st["t2"]), status, robot2_status)
        rows.append(row)
        sep = "n/a" if row.min_separation_m is None else f"{row.min_separation_m:.3f} m"
        print(f"[{n}/{len(trials)}] {trial.scenario:<15} seed {trial.seed} "
              f"layer {'on ' if trial.layer_enabled else 'off'}  {status:<9} "
              f"{elapsed:5.1f} s  min separation {sep}  recoveries {row.result.recoveries}"
              + (f"  robot2 {robot2_status}" if real else ""), flush=True)

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
    parser.add_argument("--robot1", default="",
                        help="namespace of the robot under test (multi_robot.launch.py)")
    parser.add_argument("--robot2", default="",
                        help="namespace of a real second robot; without it a box is spawned")
    args = parser.parse_args()
    if bool(args.robot1) != bool(args.robot2):
        parser.error("--robot1 and --robot2 go together (multi_robot.launch.py)")
    real = bool(args.robot2)

    trials = make_trials(args.per_scenario, args.base_seed)
    rows = _run(trials, args.robot1, args.robot2)
    trial_by_key = {(t.scenario, t.seed, t.layer_enabled): t for t in trials}
    summary = summarize_ab(rows)

    RESULTS.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = RESULTS / (f"plugin_ab_2robots_{stamp}" if real else f"plugin_ab_{stamp}")
    with open(f"{base}.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["scenario", "seed", "layer_enabled", "robot2_x", "robot2_y", "robot2_yaw",
                    "robot2_delay_s", "status", "success", "time_s", "path_length_m",
                    "final_error_m", "recoveries", "min_separation_m", "contact",
                    "robot2_status"])
        for r in rows:
            t = trial_by_key[(r.scenario, r.seed, r.layer_enabled)]
            sep = r.min_separation_m
            w.writerow([r.scenario, r.seed, r.layer_enabled, *[f"{v:.3f}" for v in t.robot2_start],
                        f"{t.robot2_delay_s:.2f}", r.status, r.result.success,
                        f"{r.result.time_s:.2f}", f"{r.result.path_length_m:.3f}",
                        f"{r.result.final_error_m:.3f}", r.result.recoveries,
                        "" if sep is None else f"{sep:.3f}",
                        sep is not None and sep < CONTACT_M, r.robot2_status])

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
    if real:
        g = SCENARIOS["crossing"].robot2_goal
        robot2_line = (
            f"- robot2: a second vision_bot (`{args.robot2}`) with its own Nav2 and "
            "RobotAvoidanceLayer, switched together with the robot under test "
            f"(`{args.robot1}`). In crossing it is sent a Nav2 goal to ({g[0]}, {g[1]}) after "
            "the seeded delay and cancelled when the trial ends; it reached that goal in "
            + ", ".join("{}/{} trials with the layer {}".format(
                *robot2_arrivals(rows, "crossing", on), "on" if on else "off")
                for on in (False, True))
            + ".")
    else:
        robot2_line = ("- robot2: a scripted box with this robot's footprint, its true pose "
                       "relayed as `/robot2/shared_pose`.")
    md = "\n".join([
        f"# RobotAvoidanceLayer A/B, {stamp}",
        "",
        f"- Configuration: {args.label}.",
        robot2_line,
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
