# vision_bot v2 architecture

v2 turns the v1 vision-guided rover into a simulated two-robot fleet: each robot maps, localizes and navigates with Nav2 on a saved map, avoids the other robot through a custom costmap layer, and takes goals from a separate fleet-management platform through a bridge node. Decisions behind each piece are numbered in `docs/DECISIONS.md`.

```
Fleet platform (robot-fleet-management: FastAPI, PostgreSQL, JWT, RBAC)
      ^ REST                                           ^
      |                                                |
 fleet_bridge /robot1                             fleet_bridge /robot2
      | NavigateToPose, map pose, nav state, battery   |
 /robot1: lidar, imu, odom -> EKF -> AMCL -> Nav2 -> twist_mux -> diff drive
 /robot2: same stack, own namespace, own TF tree, shared saved map
 v1 perception + motor_control remain, feeding twist_mux as one input

 Dashboard (Next.js): v1 panels over rosbridge; map view over the fleet API
```

## Packages and folders

| Path | What it is |
|---|---|
| `ros2_ws/src/vision_bot` | v1, unchanged: perception and motor-control nodes, v1 worlds and launch files |
| `ros2_ws/src/vision_bot_nav` | v2 robot model (lidar, IMU), navigation world and saved map, EKF / twist_mux / Nav2 / slam_toolbox configs, launch files (`nav_sim`, `slam`, `nav2`, `multi_robot`), `shared_pose` relay |
| `ros2_ws/src/vision_bot_plugins` | C++ `RobotAvoidanceLayer`, a Nav2 costmap layer that marks the other robots |
| `ros2_ws/src/fleet_bridge` | Python node connecting one robot to the fleet API |
| `benchmarks/` | Every reported number comes from a script here, raw output in `benchmarks/results/` |
| `scripts/` | Gate tools: `check_v1.sh`, map and scan checks, `seed_fleet.py`, `map_to_png.py` |
| `dashboard/` | Next.js dashboard: v1 panels plus the v2 map view and its server routes |

## One robot's stack

| Stage | Node | In | Out |
|---|---|---|---|
| Sensors (Gazebo plugins) | lidar, IMU, diff drive, ground truth | | `scan`, `imu`, `odom`, `ground_truth/pose` (truth is for benchmarks only; nothing navigates on it) |
| Odometry fusion | `ekf_filter_node` (robot_localization) | `odom` forward speed and turn rate, `imu` turn rate | `odometry/filtered`, TF `odom -> base_link` |
| Localization | AMCL | `scan`, TF, the saved map | TF `map -> odom`, `amcl_pose` |
| Mapping (instead of AMCL) | slam_toolbox | `scan`, TF | TF `map -> odom`, the map |
| Planning and control | Nav2 (NavFn planner, Regulated Pure Pursuit, behavior tree) | goal, costmaps | `cmd_vel_nav` |
| Velocity arbitration | twist_mux | see below | `cmd_vel`, the only command the drive plugin reads |
| Pose for others | `shared_pose_publisher` | TF `map -> base_link` | `shared_pose` at 10 Hz |
| Fleet connection | `fleet_bridge` | `shared_pose`, Nav2 action results | fleet telemetry, goal statuses; Nav2 goals from the fleet |

### Velocity priorities (`twist_mux.yaml`)
| Input | Topic | Priority |
|---|---|---|
| Teleop (keyboard or scripted driving) | `cmd_vel_teleop` | 100 |
| v1 line following | `cmd_vel_line` | 20 |
| Nav2 | `cmd_vel_nav` | 10 |
| Idle stop | `cmd_vel_idle` | 1 |

A person always wins; the v1 behavior, when switched on, overrides navigation; navigation is the default autonomous mover; an idle source keeps a zero command flowing. Each input times out after 0.5 s.

## Invariants (checked at every gate)
1. **One owner of the final velocity command:** every source publishes to its own topic and twist_mux publishes the only `cmd_vel`.
2. **One publisher per transform:** the EKF owns `odom -> base_link` (the drive plugin's TF output is off); AMCL owns `map -> odom` while navigating, slam_toolbox while mapping, never both; static frames come from `robot_state_publisher`.
3. **Simulation time everywhere** (`use_sim_time: true`).
4. **v1 is never broken:** `scripts/check_v1.sh` builds everything and checks the v1 sim at every gate.
5. **No invented numbers:** every metric comes from a script in `benchmarks/` with its raw output committed.

## Two robots
`multi_robot.launch.py` starts one Gazebo world and the full stack twice, under `/robot1` and `/robot2`, each in its own launch group. Each robot's `/tf` and `/tf_static` are remapped to `/robotN/tf`, so both trees can use the same frame names without colliding, and both load the same saved map. Per-robot parameters come from one shared YAML through `launch/robot_params.py` (namespace, AMCL's initial pose, which pose topic the avoidance layer watches), so the two robots cannot drift apart in tuning.

The robots cannot see each other with their lidar (the scan plane passes above the chassis), so each robot's `RobotAvoidanceLayer` reads the other robot's `shared_pose` and marks a lethal disk with a cost that decays outward, in both the local and global costmaps. `shared_pose` exists because AMCL only publishes after a filter update, which needs motion: a parked robot would otherwise vanish from the other's costmap after the layer's 2 s staleness limit.

## Fleet integration
```
browser --(map click)--> dashboard server routes --(REST, operator account)--> fleet API
fleet API <--(poll pending goals every 1.5 s; telemetry every 1 s; status reports)-- fleet_bridge
fleet_bridge --(NavigateToPose)--> Nav2
```
- **Goals**: the fleet API stores goals (`pending`, `accepted`, `succeeded`, `failed`, `canceled`). Each robot's bridge polls its pending goals, sends the oldest to Nav2, and reports `accepted` when Nav2 accepts it and the outcome when it ends. A new goal replaces the active one; the replacement is sent only after Nav2 has finished cancelling the old goal (Nav2 aborts a goal that arrives while it is cancelling). Success reported by Nav2 farther than 0.30 m from the requested goal is reported as `failed`.
- **Telemetry**: map pose, navigation state and a battery level simulated from distance driven (flagged `battery_simulated`).
- **Auth**: the bridge and the dashboard each have their own operator account (no rights to create robots or users). Clients refresh an expired access token once and log in again if the refresh is refused; tokens are never logged and never reach the browser. Credentials live only in git-ignored env files written by `scripts/seed_fleet.py`.
- **Resilience**: the bridge never exits on an API outage. Registration retries with exponential backoff (1 s doubling to 30 s), status reports stay queued in order until delivered, and rate limiting (429) and server errors (5xx) count as temporary. HTTP runs on its own thread; every ROS call stays on the ROS executor thread.

## Testing
| What | Where | Count |
|---|---|---|
| Costmap math and layer load | `vision_bot_plugins/test` (gtest, launch test) | 13 |
| Benchmarks, scripts, launch helpers | `benchmarks/tests`, `scripts/tests`, `vision_bot_nav/test` (pytest) | see CI |
| Fleet bridge client, goal handling, recovery | `fleet_bridge/test` (pytest, fake HTTP) | 56 |
| Dashboard map math, fleet client, robot choice | `dashboard/app/lib/*.test.ts` (vitest) | 29 |
| EKF regression | `benchmarks/replay_regression.py` on a recorded drive | 1 bag |
| v1 regression | `scripts/check_v1.sh` | |

`.github/workflows/v2-ci.yml` runs all of these except `check_v1.sh` (which needs Gazebo) on every push to a `v2/*` branch.
