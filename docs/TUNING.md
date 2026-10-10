# vision_bot v2: Nav2 tuning

How `ros2_ws/src/vision_bot_nav/config/nav2_params.yaml` got its values. The file started as a verbatim copy of the installed `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` (navigation2 1.1.20); every change is marked `v2:` in the file and explained here. Each round was measured with `benchmarks/nav_goals.py` (11 goals in `benchmarks/goals.yaml`, driven in order, scored against ground truth), and every number below comes from a file in `benchmarks/results/`.

## Required edits (not tuning: the defaults describe a different robot)

| Parameter | Default | Value | Why |
|---|---|---|---|
| `amcl.base_frame_id` | `base_footprint` | `base_link` | This robot's base frame. |
| `amcl.laser_max_range` | 100.0 | 8.0 | The lidar's `range_max`. |
| `amcl.set_initial_pose`, `initial_pose` | unset | true, (0, 0, 0) | Start localized at the spawn pose; the map frame equals the world frame because mapping started at the origin. |
| `bt_navigator.odom_topic`, `controller_server.odom_topic`, `velocity_smoother.odom_topic` | `odom` | `odometry/filtered` | The EKF output, not raw wheel odometry (Gate 2: EKF error 0.01 to 0.07 m vs raw 0.53 to 0.66 m final over 21.6 m). |
| `local_costmap` / `global_costmap` `robot_radius` | 0.22 | footprint polygon `[[0.11, 0.095], [0.11, -0.095], [-0.10, -0.095], [-0.10, 0.095]]` | From the URDF: chassis x -0.10 to 0.10 (camera to 0.11), wheels to y +-0.093. 0.22 is the TurtleBot3's radius. |

Velocity routing is a launch change, not a parameter: every Nav2 velocity (controller via the velocity smoother, and the behavior server's recoveries) enters twist_mux on `cmd_vel_nav` instead of the stock bringup's direct `cmd_vel` (`launch/nav2.launch.py`).

## Baseline: installed defaults plus the required edits

`benchmarks/results/nav_goals_20261010_003755.md`, commit `e8e7bc1`.

- 11 of 11 goals succeeded; mean time 27.2 s; mean final error 0.182 m (ground truth); **35 recoveries**.
- The log explains the recoveries: CPU load in the container. The behavior tree waited the default 20 ms for the controller and planner to acknowledge a request and gave up 24 times (each a recovery); the 20 Hz controller missed its rate 727 times; the 100 Hz behavior tree overran 2150 times.

## Round 1: CPU load and goal precision

`benchmarks/results/nav_goals_20261010_005351.md`, commit `1cb1128`.

| Parameter | Default | Value | Why |
|---|---|---|---|
| `bt_navigator.default_server_timeout` | 20 ms | 200 ms | Servers under load took longer than 20 ms to acknowledge; each timeout triggered a recovery. |
| `bt_navigator.bt_loop_duration` | 10 ms | 20 ms | 100 Hz ticks overran 2150 times; 50 Hz is plenty for a robot this slow. |
| `controller_server.controller_frequency` | 20 Hz | 10 Hz | 727 missed cycles; at 0.26 m/s the robot moves 2.6 cm per cycle at 10 Hz. |
| `FollowPath.debug_trajectory_details` | True | False | Debug output only costs CPU. |
| local costmap layer | `VoxelLayer` | `ObstacleLayer` | A 2D lidar has no height information; the 3D voxel grid only cost CPU. Same layer type as the global costmap. |
| `xy_goal_tolerance` (goal checker and DWB) | 0.25 m | 0.15 m | 0.25 m is longer than this 0.2 m robot. |

Result: 10 of 11; mean time 22.6 s (from 27.2); mean final error 0.110 m (from 0.182); recoveries 16 (from 35), 15 of them on `g08_west`, which aborted. Every other goal needed one recovery between them.

Kept at their defaults, deliberately:
- **Speeds** (`max_vel_x` 0.26 m/s, `max_vel_theta` 1.0 rad/s, matching velocity-smoother limits): the TurtleBot3 values, and this robot was verified to track 0.25 m/s and 1.0 rad/s exactly in Task 2's spin and straight-line tests.
- **Inflation** (`inflation_radius` 0.55 m, `cost_scaling_factor` 3.0): doorways are 1.2 to 1.4 m wide and the robot's circumscribed radius is 0.145 m, so every doorway has a low-cost centre line. No baseline or round-1 failure traced to inflation (see `g08_west` below for the one candidate).
- **AMCL**: localization error at goal ends was 0.04 to 0.24 m (the round-1 column overstates it; see "Measurement fix"). Not tuned yet.

## Startup: composition

One of twelve launches with one process per Nav2 server hung for 30 minutes at "Configuring smoother_server": the lifecycle manager's configure request was never answered by the separate `smoother_server` process. It did not recur in 8 dedicated restarts (8/8 active in 13 to 36 s), so the cause is not proven; the likely mechanism is a lost service reply between processes, a known Humble/Fast DDS failure mode. Nav2 now runs as components in one process, `nav2_container` (`use_composition:=true`, the stock Humble bringup default), which removes cross-process lifecycle calls and cuts CPU load: 8/8 starts active in 10 to 23 s. Commit `9063cca`.

## `g08_west`: the doorway stall

`g08_west` drives from the NW room (-4.2, 1.8) to (-3.0, -0.5), on the far side of the room's south wall: east along the room, out of the doorway at x = -1.2, around the wall's end at (-1.2, 1.0), and back west. It needed 13 recoveries in the baseline and aborted in round 1.

Reproduction on commit `9063cca` (`docs/images/gate3_g08_diagnosis.png`): the global plan is fine, and the robot follows it everywhere except the doorway. It spent about 97 of 130 s within 0.25 m of (-1.35, 1.75), with 8 "Failed to make progress" aborts and spin, wait, and back-up recoveries, before following the arc around the wall end. The progress checker requires 0.5 m of movement every 10 s.

Second reproduction (same commit, command trace kept: `benchmarks/results/diagnostics/g08_doorway_trace_20261009.json`, plot `docs/images/gate3_g08_diagnosis_run2.png`): succeeded after 113 s, 83 s of it inside the doorway. Of the 2244 `cmd_vel` samples there, 76% were turn-in-place commands (mean angular speed 0.17 rad/s, 69 sign flips), 12% forward, 8% near zero, and 4% backward (the back-up recovery); mean linear speed 0.014 m/s. The robot first sat facing east, along the path, for about 25 s, then turned to face south toward the wall end and stayed that way for about 50 s before moving on.

Diagnosis: a DWB local minimum at a U-turn. DWB sees only the part of the global plan inside its 3 x 3 m local costmap; here that part ends just past the wall end, so its goal-attraction critics (GoalAlign and GoalDist, weight 24 each) pull the robot south toward the wall end while its path critics (PathAlign and PathDist, weight 32 each) pull it east along the arc. The scores cancel, the robot dithers in place, and the progress checker trips.

_Round 2 (the fix for the doorway stall) and the final numbers are added below once measured._

## Measurement fix

`amcl_pose` is published only after an AMCL filter update (every 0.25 m or 0.2 rad of motion). Until commit `03c83c1`, `nav_goals.py` compared the latest estimate with the robot's current true position, which overstated AMCL's error whenever the robot had moved since the last update (`g02_se_room` in round 1: 0.356 m reported while the robot finished 0.033 m from the goal). It now compares each estimate with ground truth at the estimate's own timestamp. The baseline and round-1 files keep the old column.
