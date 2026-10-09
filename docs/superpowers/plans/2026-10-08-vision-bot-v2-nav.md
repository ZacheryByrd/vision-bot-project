# vision_bot v2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend the shipped vision_bot into two namespaced Nav2 robots on a shared SLAM-built map, with a custom C++ costmap layer and a bridge so goals clicked on the dashboard map flow through the fleet API to the right robot.

**Architecture:** New packages (`vision_bot_nav`, `vision_bot_plugins`, `fleet_bridge`) and a `benchmarks/` directory sit beside the untouched v1 packages. `twist_mux` arbitrates velocity; the EKF owns `odom -> base_link`; AMCL owns `map -> odom`. Pure logic (metrics, cost function, API client, map transform) is separated from ROS plumbing so it is unit tested; simulation behavior is proven by benchmark scripts whose raw output is committed.

**Tech Stack:** ROS2 (distro found in Task 0), Gazebo (Classic or new, found in Task 0), Nav2, slam_toolbox, robot_localization, twist_mux, C++17 + gtest, Python 3 + pytest, Next.js/TypeScript (existing dashboard), GitHub Actions.

**Spec:** `docs/superpowers/specs/2026-10-08-vision-bot-v2-nav-design.md` (runbook with pitfalls: `vision_bot_v2_nav_plan.md`). Read both before Task 0.

## Global Constraints

- Every simulated node sets `use_sim_time: true`.
- Exactly one node owns the final velocity command: `twist_mux` output only. Inputs: `cmd_vel_teleop` priority 100, `cmd_vel_line` priority 20, `cmd_vel_nav` priority 10, each timeout 0.5 s.
- Exactly one publisher per TF transform: EKF owns `odom -> base_link` (diff-drive TF output disabled); AMCL owns `map -> odom` during navigation; slam_toolbox owns it only during mapping.
- v1 is never broken: v1 packages change only by launch-file remaps; `scripts/check_v1.sh` passes at every gate.
- Lidar: `sensor_msgs/LaserScan` on `scan`, about 360 samples, 0.12 to 8 m, 10 Hz, with noise. IMU on `imu` with noise.
- EKF: `two_d_mode: true`, about 30 Hz, `odom0` (vx, yaw rate), `imu0` (yaw rate).
- Never write Nav2 params from memory: copy the installed `nav2_bringup` params and diff.
- No reported number without a script in `benchmarks/` and raw output in `benchmarks/results/`. Unmeasured means "not measured yet".
- Benchmarks: at least 10 goals/trials per condition, same goals and seeds across A/B conditions. Goal-success target is 9 of 10; record the actual rate either way.
- Fleet bridge: telemetry about 1 Hz; goal polling every 1 to 2 s unless the API offers WebSocket/SSE; credentials only from environment variables; `.env.example` committed, `.env` never; tokens never logged.
- Battery is simulated and labeled so.
- One git branch per task (`v2/task-N-<name>`), small commits, no force-push, no `build/ install/ log/`, large bags via Git LFS or regenerated.
- **Every task ends at a gate:** run the gate checks, append an entry to `docs/INTERVIEW_NOTES.md` (what was built, why, what broke and how it was diagnosed), report to Zach with the evidence, and **stop until Zach says go.**
- Paths below are relative to the colcon workspace `src/` directory recorded in `docs/V2_DISCOVERY.md`.

## Review Focus

Inputs the spec implies but its gates do not directly exercise, most likely first:

1. Fleet API unreachable at bridge startup: bridge keeps retrying with backoff and does not exit (pinned in Task 6).
2. Goal clicked outside the map or on an occupied cell: Nav2 rejects it, the bridge reports `failed`, the dashboard shows the failure instead of hanging (Tasks 6, 7).
3. Another robot goes offline: its last pose goes stale and must stop blocking paths after `max_pose_age_s` (Task 4).
4. Two goals sent to one robot in quick succession: the newer goal cancels the active one and only the newer is reported as running (Task 6).
5. Access token expires mid-request, or the refresh token has already rotated: one refresh and one retry, then re-login, never a loop (Task 6).

---

## File Structure

| Path | Responsibility |
|---|---|
| `docs/V2_DISCOVERY.md`, `docs/INTERVIEW_NOTES.md`, `docs/DECISIONS.md`, `docs/TUNING.md`, `docs/BENCHMARKS.md`, `docs/ARCHITECTURE.md` | Findings, explanations, decisions, reproduction steps |
| `scripts/check_v1.sh` | v1 regression check |
| `vision_bot_nav/` | Launch files, URDF/xacro, worlds, maps, params (ekf, slam, nav2, twist_mux), RViz configs |
| `vision_bot_plugins/` | `include/.../avoidance_cost.hpp`, `src/avoidance_cost.cpp` (pure function), `src/robot_avoidance_layer.cpp` (ROS plumbing), `plugins.xml`, `test/` |
| `fleet_bridge/fleet_bridge/` | `client.py` (HTTP + auth), `goal_manager.py` (goal state machine), `bridge_node.py` (ROS plumbing), `tests/` |
| `benchmarks/` | `metrics.py` (pure stats), `odom_drift.py`, `nav_goals.py`, `plugin_ab.py`, `goals.yaml`, `results/`, `tests/` |
| `<dashboard>/src/lib/mapTransform.ts` (+ test) | Pixel/world conversion; `<dashboard>` path from discovery |

---

### Task 0: Discovery, dependencies, v1 regression check

**Files:**
- Create: `docs/V2_DISCOVERY.md`, `docs/INTERVIEW_NOTES.md`, `docs/DECISIONS.md`, `scripts/check_v1.sh`

**Interfaces:**
- Produces: the recorded facts every later task reads: ROS distro, Gazebo stack (Classic or new), v1 package/node/topic/frame names, URDF wheel and body dimensions, dashboard connection details and toggle mechanism, fleet repo path and the exact auth and robot/goal/telemetry endpoints with request/response schemas. `scripts/check_v1.sh` exits 0 on pass, non-zero on fail.

- [ ] **Step 1:** Run the environment commands from runbook section 1.1 and paste output into `docs/V2_DISCOVERY.md`.
- [ ] **Step 2:** Decide Classic vs new Gazebo from the v1 launch file, URDF, and world (runbook section 1.2). If the distro cannot run the repo's stack, stop and ask Zach.
- [ ] **Step 3:** With v1 running, record `ros2 topic list -t`, `ros2 node list`, and `ros2 run tf2_tools view_frames` output; record who publishes `odom -> base_link`.
- [ ] **Step 4:** Locate the fleet repo (`robot-fleet-managment-system`). If it is not in the workspace, ask Zach for its path. List every endpoint v2 will use with request/response schemas. Mark any missing endpoint (per-robot goals, goal status) as "needs addition, propose to Zach".
- [ ] **Step 5:** Verify each dependency exists for the distro (`apt-cache policy ros-$ROS_DISTRO-<pkg>`) and install: slam-toolbox, navigation2, nav2-bringup, robot-localization, twist-mux, rviz2, tf2-tools, plus `ros-gz-*` if the new Gazebo. Record in `docs/DEPENDENCIES.md`.
- [ ] **Step 6:** Write `scripts/check_v1.sh`: build, launch the v1 sim headless, assert the perception and motor-control nodes appear in `ros2 node list` and the camera topic publishes (`ros2 topic hz` shows a rate), then shut down and exit 0; otherwise exit 1.
- [ ] **Step 7:** Run `colcon build` from clean, then `scripts/check_v1.sh`. Expected: build succeeds, script exits 0. Record the baseline.
- [ ] **Step 8:** Commit on `v2/task-0-discovery`. Gate: report to Zach, stop.

---

### Task 1: `vision_bot_nav` scaffold, sensors, world, `twist_mux`

**Files:**
- Create: `vision_bot_nav/` (package files), `urdf/vision_bot_nav.urdf.xacro`, `worlds/nav_world.*` (extension per Gazebo stack), `config/twist_mux.yaml`, `launch/nav_sim.launch.py`, `rviz/nav.rviz`
- Modify: v1 launch files only if a remap is impossible any other way (justify in commit)

**Interfaces:**
- Produces: launch args `namespace` (default empty), `world`, `use_sim_time` (default true); topics `scan`, `imu`, `odom`, `cmd_vel_teleop`, `cmd_vel_line`, `cmd_vel_nav`; the final velocity topic is whatever the drive plugin consumes (from Task 0).

- [ ] **Step 1:** Create the package. The xacro includes the v1 robot description and adds a lidar link/sensor and an IMU link/sensor per the Global Constraints, using the plugin set for the stack found in Task 0. Disable the diff-drive plugin's TF publishing; keep its `odom` topic.
- [ ] **Step 2:** Create `nav_world`: enclosed about 10 x 10 m, 2 to 3 rooms/corridors, several box and cylinder obstacles with distinct shapes and no symmetric layout.
- [ ] **Step 3:** Write `config/twist_mux.yaml` with the three inputs, priorities, and 0.5 s timeouts from Global Constraints; verify the message type (Twist vs TwistStamped) against the installed twist_mux and drive plugin.
- [ ] **Step 4:** Write `nav_sim.launch.py`: sim, spawn, `robot_state_publisher`, bridge (new Gazebo only), `twist_mux`, optional teleop to `cmd_vel_teleop`. Remap v1 `motor_control_node` output to `cmd_vel_line` in the launch file. If v1 publishes zero velocity when idle, gate that output behind line-follow mode (see runbook pitfalls) and record in `docs/DECISIONS.md`.
- [ ] **Step 5 (gate checks):**
  - `ros2 topic hz /scan` shows about 10 Hz; `ros2 topic hz /imu` shows the configured rate.
  - RViz screenshot: scan points overlay the world walls.
  - `view_frames`: every frame has one parent; launch log has no TF warnings.
  - Publish to `cmd_vel_line`: robot moves. Publish to `cmd_vel_teleop` at the same time: robot follows teleop.
  - `scripts/check_v1.sh` exits 0.
- [ ] **Step 6:** Commit on `v2/task-1-sensors`. Gate: report with the screenshot, stop.

---

### Task 2: Ground truth, EKF, SLAM, saved map, drift benchmark

**Files:**
- Create: `config/ekf.yaml`, `config/slam_toolbox.yaml`, `launch/slam.launch.py`, `maps/nav_world.yaml` + `.pgm`, `benchmarks/metrics.py`, `benchmarks/odom_drift.py`, `benchmarks/tests/test_metrics.py`
- Modify: `urdf/vision_bot_nav.urdf.xacro` (ground-truth publisher; odometry noise/slip if raw odom is too perfect)

**Interfaces:**
- Produces: `benchmarks/metrics.py` with `@dataclass Pose2D(x: float, y: float, yaw: float)`, `@dataclass ErrorStats(mean: float, max: float, final: float)`, and `position_errors(estimate: list[Pose2D], truth: list[Pose2D]) -> ErrorStats` (Euclidean xy error per index; lists must be equal length, else `ValueError`). Ground truth topic `ground_truth/pose` used only by benchmarks.

- [ ] **Step 1: Write failing tests** in `benchmarks/tests/test_metrics.py`: `test_identical_paths_zero_error` (all stats 0.0); `test_known_offset` (estimate shifted +3 m in x, 4 m in y at final sample only, truth all at origin, three samples: mean 5/3, max 5.0, final 5.0); `test_length_mismatch_raises` (`ValueError`).
- [ ] **Step 2:** Run `pytest benchmarks/tests/test_metrics.py -v`. Expected: FAIL (module missing).
- [ ] **Step 3:** Implement `position_errors` in `benchmarks/metrics.py`. Run the tests. Expected: PASS.
- [ ] **Step 4:** Add the ground-truth pose publisher for the stack found in Task 0. If raw odometry error stays under a few millimeters over a 20 m route, add noise/slip and record the parameters in `docs/DECISIONS.md`.
- [ ] **Step 5:** Write `ekf.yaml` per Global Constraints; the EKF publishes `odom -> base_link` and filtered odometry. Write `slam_toolbox.yaml` (online async; `odom_frame`, `base_frame`, `map_frame`, `scan_topic`, `use_sim_time`) and `slam.launch.py`.
- [ ] **Step 6:** Drive the world by teleop; save with `ros2 run nav2_map_server map_saver_cli -f maps/nav_world`.
- [ ] **Step 7:** Write `benchmarks/odom_drift.py`: drive a fixed route, record raw odometry, EKF output, and ground truth, compute `position_errors` for both estimates, write `benchmarks/results/odom_drift_<timestamp>.csv` plus a plot.
- [ ] **Step 8 (gate checks):** map image matches the world (RViz + PGM); `map -> odom -> base_link` valid with no TF errors during SLAM; run `python benchmarks/odom_drift.py` and confirm the CSV exists with raw vs EKF numbers; `scripts/check_v1.sh` exits 0.
- [ ] **Step 9:** Commit on `v2/task-2-slam-ekf`. Gate: report map image and drift table, stop.

---

### Task 3: Nav2 on the saved map, goal benchmark

**Files:**
- Create: `config/nav2_params.yaml`, `launch/nav2.launch.py`, `benchmarks/goals.yaml`, `benchmarks/nav_goals.py`, `docs/TUNING.md`
- Modify: `benchmarks/metrics.py`, `benchmarks/tests/test_metrics.py`

**Interfaces:**
- Consumes: `Pose2D`, `position_errors` (Task 2).
- Produces: in `metrics.py`, `@dataclass GoalResult(goal_id: str, success: bool, time_s: float, path_length_m: float, final_error_m: float, recoveries: int)`, `@dataclass GoalSummary(n: int, success_rate: float, mean_time_s: float, mean_final_error_m: float, total_recoveries: int)`, and `summarize_goals(results: list[GoalResult]) -> GoalSummary` (time and error means over successful goals only; empty list returns `n=0`, `success_rate=0.0`, other fields 0.0/0).

- [ ] **Step 1: Write failing tests:** `test_summary_counts_success_rate` (3 of 4 succeed, rate 0.75); `test_means_use_successes_only` (a failed goal with `time_s=999` does not move the mean); `test_empty_results` (`n == 0`, `success_rate == 0.0`).
- [ ] **Step 2:** Run the tests. Expected: FAIL. Implement `summarize_goals`. Expected: PASS.
- [ ] **Step 3:** Copy the installed `nav2_bringup` params into `nav2_params.yaml` and edit: footprint/radius from the URDF, odom topic = EKF output, scan topic, `use_sim_time`, map path. Route the final velocity output to `cmd_vel_nav` (verify each hop with `ros2 topic info -v`). Localization via `map_server` + `amcl` with an initial pose.
- [ ] **Step 4:** Tune costmap inflation, speeds, and goal tolerances; record values and reasons in `docs/TUNING.md`.
- [ ] **Step 5:** Write `benchmarks/goals.yaml` (at least 10 goals spread across rooms) and `nav_goals.py`: send each through `NavigateToPose` with a per-goal timeout, build `GoalResult` using ground truth for `final_error_m`, write CSV and a markdown summary from `summarize_goals` to `benchmarks/results/`.
- [ ] **Step 6 (gate checks):** a goal sent from RViz is reached; `python benchmarks/nav_goals.py` completes 10 goals and the real success rate is recorded; teleop still overrides Nav2; `scripts/check_v1.sh` exits 0. If success is below 9 of 10, keep tuning and record what changed.
- [ ] **Step 7:** Commit on `v2/task-3-nav2`. Gate: report CSV and a GIF of a run, stop.

---

### Task 4: `RobotAvoidanceLayer` and A/B benchmark

**Files:**
- Create: `vision_bot_plugins/` (`CMakeLists.txt`, `package.xml`, `plugins.xml`), `include/vision_bot_plugins/avoidance_cost.hpp`, `src/avoidance_cost.cpp`, `src/robot_avoidance_layer.cpp`, `test/test_avoidance_cost.cpp`, `test/test_plugin_load.py`, `benchmarks/plugin_ab.py`
- Modify: `config/nav2_params.yaml` (register the layer)

**Interfaces:**
- Consumes: `GoalResult`, `summarize_goals` (Task 3).
- Produces: `unsigned char computeAvoidanceCost(double distance_m, double robot_radius_m, double cost_scaling)` in namespace `vision_bot_plugins`. Layer class `vision_bot_plugins::RobotAvoidanceLayer : public nav2_costmap_2d::Layer`, params `robot_pose_topics` (string array), `robot_radius` (double), `cost_scaling` (double), `max_pose_age_s` (double), `enabled` (bool), all declared with defaults. Also `bool isPoseFresh(double pose_stamp_s, double now_s, double max_age_s)` in the same header.

- [ ] **Step 1: Write failing gtest cases** in `test/test_avoidance_cost.cpp`: `LethalAtCenter` (`computeAvoidanceCost(0.0, 0.4, 3.0) == 254`); `LethalAtRadius` (`(0.4, 0.4, 3.0) == 254`); `DecaysBeyondRadius` (`(0.4 + 1.0/3.0, 0.4, 3.0) == 92`, i.e. `floor(252 * exp(-1))`); `ZeroFarAway` (`(10.0, 0.4, 3.0) == 0`); `MonotonicNonIncreasing` (cost never rises as distance grows over 0 to 5 m in 0.05 steps); `StalePoseIgnored` (`isPoseFresh(0.0, 5.0, 2.0) == false`, `isPoseFresh(4.5, 5.0, 2.0) == true`).
- [ ] **Step 2:** Run `colcon test --packages-select vision_bot_plugins`. Expected: FAIL.
- [ ] **Step 3:** Implement the two functions in `avoidance_cost.cpp`: cost 254 when `distance_m <= robot_radius_m`, else `floor(252 * exp(-cost_scaling * (distance_m - robot_radius_m)))`.
- [ ] **Step 4:** Implement the layer in `robot_avoidance_layer.cpp` by following the installed `nav2_costmap_2d` layer interface (read an existing layer as reference): subscribe to each topic in `robot_pose_topics`, keep the latest pose per topic under a mutex, skip poses failing `isPoseFresh`, transform to the costmap frame, mark cells in `updateBounds`/`updateCosts` using `computeAvoidanceCost`. Export via `plugins.xml` and `pluginlib_export_plugin_description_file`.
- [ ] **Step 5:** Write `test/test_plugin_load.py` (launch test): load the layer in a costmap node and assert no errors in the log.
- [ ] **Step 6:** Register the layer in `nav2_params.yaml` (local and global costmaps). Run `colcon test`. Expected: all PASS, no warnings treated as errors.
- [ ] **Step 7:** Write `benchmarks/plugin_ab.py`: same goals and seeds with the layer disabled vs enabled, scenarios crossing paths and a robot parked in a corridor (stationary or scripted second robot until Task 5); metrics: `summarize_goals` plus minimum distance to the other robot. At least 10 trials per condition; write CSV and markdown to `benchmarks/results/`.
- [ ] **Step 8 (gate checks):** `colcon test` passes; Nav2 lifecycle logs show the layer loaded; layer visible in RViz; A/B results saved (report honestly even if the layer is not better); `scripts/check_v1.sh` exits 0.
- [ ] **Step 9:** Commit on `v2/task-4-plugin`. Gate: report the A/B table, stop.

---

### Task 5: Second robot and namespaces

**Files:**
- Create: `launch/multi_robot.launch.py`, `config/nav2_params_robot.yaml` (namespace-aware variant if the installed reference needs one)
- Modify: `benchmarks/nav_goals.py`, `benchmarks/plugin_ab.py`, `rviz/nav.rviz` (take a `--namespace` argument)

**Interfaces:**
- Consumes: Task 1 to 4 launch args and params.
- Produces: namespaces `robot1`, `robot2`, each with its own Nav2, AMCL, EKF, `twist_mux`, `robot_state_publisher`, and TF tree (`/tf` and `/tf_static` remapped to namespaced topics), sharing `maps/nav_world.yaml`; pose topics `robot1/amcl_pose` and `robot2/amcl_pose` consumed by the avoidance layer.

- [ ] **Step 1:** Read the installed `nav2_bringup` multi-robot reference launch and base `multi_robot.launch.py` on it; give each robot a unique model/sensor name, spawn pose, and AMCL initial pose.
- [ ] **Step 2:** Point each robot's `robot_pose_topics` at the other robot's pose topic.
- [ ] **Step 3:** Add `--namespace` to both benchmarks and run `nav_goals.py` for each robot independently.
- [ ] **Step 4 (gate checks):** `ros2 node list` shows two stacks under `/robot1` and `/robot2`; a velocity command to one robot never moves the other; both localize on the shared map; each reaches 10 goals (record rates); rerun `plugin_ab.py` with two real robots and save results; `scripts/check_v1.sh` exits 0.
- [ ] **Step 5:** Commit on `v2/task-5-multirobot`. Gate: report, stop.

---

### Task 6: `fleet_bridge`

**Files:**
- Create: `fleet_bridge/` package with `fleet_bridge/client.py`, `fleet_bridge/goal_manager.py`, `fleet_bridge/bridge_node.py`, `tests/test_client.py`, `tests/test_goal_manager.py`, `.env.example`
- Modify (only after Zach approves the proposal from Task 0): the fleet repo, to add missing endpoints with tests and RBAC

**Interfaces:**
- Produces in `client.py`: `class FleetClient(base_url: str, username: str, password: str, http: HttpSession)` with `login() -> None`, `register_robot(robot_id: str, name: str) -> dict`, `post_telemetry(robot_id: str, pose: tuple[float, float, float], nav_state: str, battery_pct: float, battery_simulated: bool = True) -> None`, `fetch_pending_goal(robot_id: str) -> Goal | None`, `report_goal_status(goal_id: str, status: str) -> None` (status in `accepted | succeeded | failed | canceled`). `@dataclass Goal(goal_id: str, x: float, y: float, yaw: float)`. Every call retries once after a refresh on HTTP 401 and falls back to `login()` if refresh fails. Endpoint paths and field names come from `docs/V2_DISCOVERY.md`.
- Produces in `goal_manager.py`: `class GoalManager(send: Callable[[Goal], None], cancel: Callable[[str], None], report: Callable[[str, str], None])` with `on_new_goal(goal: Goal) -> None`, `on_result(goal_id: str, status: str) -> None`, `active_goal_id -> str | None`.

- [ ] **Step 1: Write failing tests** with a fake `HttpSession`:
  - `test_login_stores_tokens`
  - `test_expired_token_refreshes_once_and_retries` (first call 401, refresh ok, retry ok; exactly one refresh)
  - `test_rotated_refresh_token_is_stored` (the new refresh token replaces the old one)
  - `test_refresh_failure_triggers_relogin` (refresh 401, then login, then retry; no loop)
  - `test_api_down_raises_fleet_unavailable_not_crash` (connection error surfaces as `FleetUnavailable`)
  - `test_tokens_never_logged` (captured log contains neither token)
- [ ] **Step 2:** Run `pytest fleet_bridge/tests/test_client.py -v`. Expected: FAIL. Implement `FleetClient`. Expected: PASS.
- [ ] **Step 3: Write failing tests** in `test_goal_manager.py`: `test_new_goal_sends_and_sets_active`; `test_second_goal_cancels_first` (cancel called with first id, send called with second, active is second); `test_rejected_goal_reports_failed`; `test_result_for_stale_goal_ignored`. Run (FAIL), implement `GoalManager`, run (PASS).
- [ ] **Step 4:** Implement `bridge_node.py` (parameters `namespace`, `robot_id`, `api_base_url`; env `FLEET_USER`, `FLEET_PASSWORD`): register on startup with exponential backoff if the API is down (never exit); every 1 s read the pose in the robot's `map` frame and post telemetry with simulated battery draining by distance; every 1 to 2 s poll `fetch_pending_goal` and feed `GoalManager`; map `NavigateToPose` action responses to statuses; a Nav2 rejection reports `failed`.
- [ ] **Step 5:** If an endpoint is missing, write the proposal in `docs/DECISIONS.md` and ask Zach before touching the fleet repo; once approved add the endpoints with pytest tests and RBAC matching existing routes.
- [ ] **Step 6 (gate checks, with the real API):** bridge registers and posts telemetry for both robots; kill and restart the API, bridge recovers; force a token expiry, bridge refreshes; send an unreachable goal, status shows `failed`; `scripts/check_v1.sh` exits 0.
- [ ] **Step 7:** Commit on `v2/task-6-fleet-bridge`. Gate: report, stop.

---

### Task 7: Dashboard map view and click-to-goal

**Files:**
- Create: `<dashboard>/src/lib/mapTransform.ts`, `<dashboard>/src/lib/mapTransform.test.ts`, the map-view component, a one-time PGM-to-PNG conversion script `scripts/map_to_png.py`
- Modify: the dashboard page that hosts the view (path from discovery)

**Interfaces:**
- Produces: `type MapMeta = { resolution: number; origin: [number, number]; widthPx: number; heightPx: number }`; `pixelToWorld(px: number, py: number, meta: MapMeta): { x: number; y: number }` where `py` is measured from the image top; `worldToPixel(x: number, y: number, meta: MapMeta): { px: number; py: number }`; `isInsideMap(px: number, py: number, meta: MapMeta): boolean`.

- [ ] **Step 1: Write failing tests:** with `meta = { resolution: 0.05, origin: [-5, -5], widthPx: 200, heightPx: 200 }`: `pixelToWorld(0, 200)` equals `{ x: -5, y: -5 }` (bottom-left); `pixelToWorld(200, 0)` equals `{ x: 5, y: 5 }`; `pixelToWorld(100, 100)` equals `{ x: 0, y: 0 }`; `worldToPixel` inverts `pixelToWorld` for the three points; `isInsideMap(-1, 10)` and `isInsideMap(10, 201)` are false. Use the dashboard's existing test runner.
- [ ] **Step 2:** Run the tests. Expected: FAIL. Implement: `x = origin[0] + px * resolution`, `y = origin[1] + (heightPx - py) * resolution`, and the inverse. Run. Expected: PASS.
- [ ] **Step 3:** Run `scripts/map_to_png.py` once to convert `maps/nav_world.pgm` to PNG; copy the PNG and `MapMeta` values (from `nav_world.yaml`) into the dashboard assets.
- [ ] **Step 4:** Build the map view: render the PNG, overlay each robot's pose and heading arrow with status from fleet telemetry, accept a click (ignore clicks where `isInsideMap` is false), choose robot or "nearest idle", POST the goal through the fleet API (not rosbridge), show `failed` status in the UI.
- [ ] **Step 5 (gate checks):** existing dashboard features (MJPEG, telemetry, manual/auto toggle, reconnect) still work; a click moves the chosen robot to the clicked point and the fleet platform shows status transitions; a click on a wall shows a failed status; `scripts/check_v1.sh` exits 0.
- [ ] **Step 6:** Commit on `v2/task-7-dashboard`. Gate: report with a GIF of click-to-motion, stop.

---

### Task 8: CI, rosbag regression, documentation

**Files:**
- Create: `.github/workflows/v2-ci.yml`, `benchmarks/replay_regression.py`, `benchmarks/baseline.json`, `README.md` section for v2, `docs/ARCHITECTURE.md`, `docs/BENCHMARKS.md`

**Interfaces:**
- Consumes: `position_errors` (Task 2). Produces: `replay_regression.py --bag <path> --threshold-m <float>` exiting 0 if EKF mean position error vs ground truth in the bag is at or below the threshold, else 1; `baseline.json` records the measured baseline and the margin used.

- [ ] **Step 1:** Record a small compressed bag of `/scan`, `/imu`, `/odom`, `/tf_static`, and ground truth from a standard run (Git LFS or release artifact).
- [ ] **Step 2:** Write `replay_regression.py`; measure the baseline, set the threshold to baseline plus a documented margin in `baseline.json`.
- [ ] **Step 3:** Write `v2-ci.yml` using the ROS Docker image matching the distro: `colcon build`, `colcon test`, bridge pytest, benchmark metrics pytest, dashboard tests, replay regression. No GUI Gazebo in CI.
- [ ] **Step 4:** Push a branch with a deliberately tight threshold. Expected: CI fails red. Restore the threshold. Expected: CI green. Record both runs.
- [ ] **Step 5:** Write the README v2 section (architecture diagram from spec section 3, demo GIF, exact run commands, benchmark tables copied from `benchmarks/results/`, limitations, v1 vs v2) and `docs/BENCHMARKS.md` (how to reproduce each number).
- [ ] **Step 6:** Finish `docs/INTERVIEW_NOTES.md` with a Q&A section: EKF configuration rationale, how AMCL localizes, what the cost function computes, how token refresh works, why `twist_mux` priorities are ordered as they are.
- [ ] **Step 7 (gate checks):** CI green and proven able to fail; a fresh clone follows the README to a working demo; every README number maps to a file in `benchmarks/results/`.
- [ ] **Step 8:** Commit on `v2/task-8-ci-docs`. Final report to Zach with the resume bullets from runbook section 11 filled in with measured numbers only.
