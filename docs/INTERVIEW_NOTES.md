# vision_bot v2: Interview Notes

Plain-English notes on what was built, why, what broke, and how it was diagnosed. One entry per gate. Read these before approving the next phase; the final task (plan Task 8) turns them into a Q&A section.

---

## Gate 0: Discovery, dependencies, v1 regression check (2026-10-08)

### What was done
- Recorded the real environment instead of assuming it: ROS 2 Humble on Ubuntu 22.04 inside the `vision_bot_dev:humble` Docker image, Gazebo Classic 11.10.2, Python 3.10 (`docs/V2_DISCOVERY.md`).
- Ran the v1 simulation headless and captured the live ROS graph: every node, every topic with its message type, who publishes and subscribes to `/cmd_vel` and `/tf`, and the TF tree.
- Read the fleet platform's live OpenAPI schema and its route code to list the exact endpoints, request/response fields, and which role each endpoint requires.
- Added the v2 navigation packages (slam_toolbox, Nav2, robot_localization, twist_mux) to the Docker image and recorded versions in `docs/DEPENDENCIES.md`.
- Wrote `scripts/check_v1.sh`, the regression check that runs at every gate from now on.

### Why Gazebo Classic and not the newer Gazebo
The Docker base image ships both simulators, so "what is installed" does not answer the question. The repo does: the launch file uses `gazebo_ros` and `spawn_entity.py`, the robot uses `libgazebo_ros_*` plugins, and the worlds are `.world` files. Those are all Gazebo Classic. Humble supports Classic, so there is no reason to migrate, and every later step uses the Classic plugin set (for example `libgazebo_ros_ray_sensor.so` for the lidar).

### Surprise: v1 had no odometry at all
- **Expected:** reading the robot description, the diff-drive plugin names an `odom` topic, so the plan assumed it published odometry and the `odom -> base_link` transform, and that v2 would need to switch the transform off so the EKF could own it.
- **Observed:** with the sim running, `ros2 topic info /odom` said "Unknown topic", `ros2 topic info -v /tf` showed only `robot_state_publisher`, and `tf2_echo odom base_link` said the `odom` frame does not exist.
- **Cause:** in ROS 2's `gazebo_plugins`, the diff-drive options `publish_odom`, `publish_odom_tf`, and `publish_wheel_tf` all default to false. v1 never set them, and v1 never needed odometry because it steers purely from the camera. The `odometry_topic` tag in the URDF only names the topic; it does not turn publishing on.
- **Why it matters:** the EKF, SLAM, and Nav2 all need wheel odometry. Task 1 will turn on `publish_odom`, keep `publish_odom_tf` off (the EKF owns that transform from Task 2), and publish the wheel joints. Lesson: confirm a topic exists with `ros2 topic list` before designing around it.

### Smaller things that broke and how they were diagnosed
- **The runbook's version check reported the wrong simulator.** `gz sim --version || ... || gazebo --version` stops at the first command that works, and `gz sim` (Gazebo Fortress) exists in the image even though the project does not use it. Ran `gazebo --version` separately to get Classic 11.10.2.
- **`set -u` broke ROS's setup script.** Bash's "error on unset variables" option makes `/opt/ros/humble/setup.bash` fail on `AMENT_TRACE_SETUP_FILES`. Removed it.
- **colcon rejected `--log-base` after `build`.** It is a global option and goes before the verb: `colcon --log-base DIR build ...`.

### Fleet API facts that affect later tasks
- Login is an OAuth2 form post where `username` is the user's **email**.
- Creating a robot is **admin-only**; reading robots and posting telemetry allow admin or operator. That raises a question for Zach about the bridge's service account (see `docs/V2_DISCOVERY.md`, "Open question").
- The refresh endpoint hands back a new refresh token every time (rotation), but the server does not revoke the old one. The bridge still stores the newest token, which is the correct client behavior.
- Token lifetimes are environment settings, so the "expired token" gate check in Task 6 can run the API with a 1-minute access token instead of waiting an hour.

### How `scripts/check_v1.sh` works and why it is built that way
- It builds the workspace **from scratch into a temporary directory**, so it never depends on or overwrites the everyday `ros2_ws/build` and `install` folders. Every run is a clean-build test.
- It launches the v1 sim headless, waits for `perception_node` and `motor_control_node` to appear, and checks that the camera topic reports a publish rate. Pass exits 0, fail exits 1 and prints the launch log.
- It sets `ROS_LOCALHOST_ONLY=1`. Containers on the same Docker network can discover each other's ROS nodes, so without this a sim running in another container could make a broken build look like a pass.
- It was run once with a deliberately impossible expectation (a node name that does not exist) to prove it can fail, then run normally to prove it passes.

### Measured baseline
- Clean build: 1 package in 5.3 s, no errors (one old setuptools deprecation warning).
- v1 unit tests: 23 of 23 pass. Lint (same settings as CI): clean.
- `check_v1.sh`: passes (both v1 nodes up, camera publishing at 14.4 Hz headless); with an impossible node name it fails with exit 1, so the check is proven able to fail.
- The camera is configured for 30 Hz but measures 14 to 17 Hz in the headless container because Gazebo renders the camera in software there. That is why the check asserts "publishing", not a rate.

---

## Gate 1: Sensors, navigation world, velocity mux (2026-10-08)

### What was built
- **`vision_bot_nav` package** (`ros2_ws/src/vision_bot_nav`): the v2 robot model, a navigation world, the `twist_mux` config, a launch file, and an RViz config. The v1 package is untouched.
- **Robot model** (`urdf/vision_bot_nav.urdf.xacro`): the v1 rover plus a 2D lidar (360 samples, 0.12 to 8 m, 10 Hz, 1 cm noise) on a short mast and an IMU (50 Hz, small gyro and accelerometer noise). Odometry is now published (v1 had none; see Gate 0).
- **World** (`worlds/nav_world.world`): a walled 10 x 10 m space with two rooms with doorways, a corridor, an alcove, and six obstacles of different shapes and sizes placed so no two spots look alike to the lidar. Symmetric layouts make localization guess the wrong room.
- **Velocity arbitration**: every source publishes to its own topic and `twist_mux` forwards the highest-priority one that is still active: teleop (100) > v1 line following (20) > navigation (10) > idle zero (1). Only `twist_mux` publishes `cmd_vel`, the topic the drive plugin listens to.
- **Launch** (`launch/nav_sim.launch.py`): Gazebo, robot spawn, `robot_state_publisher`, `twist_mux`, the idle-stop node, optional RViz, and the v1 nodes only when `line_follow:=true`. A `namespace` argument pushes everything into e.g. `/robot1`.
- **Two checking tools** in `scripts/`: `scan_overlay.py` and `mux_check.py` (below).

### Why these choices
- **Copy, not include, of the v1 robot model.** The drive plugin block had to change, xacro cannot remove a block from an included file, and v1's file must stay as it is.
- **Encoder odometry.** Gazebo's drive plugin can report either the robot's true pose or a pose integrated from wheel motion. "True pose" sounds better but would make odometry perfect, so the EKF in Task 2 would have nothing to improve on. Wheel-integrated odometry drifts the way a real robot's does.
- **Wheel transforms from `robot_state_publisher`.** Gazebo publishes the wheel joint angles and `robot_state_publisher` turns them into transforms, so one node owns every frame inside the robot. The odom transform is left for the EKF.
- **v1 nodes only on request.** v1's motor controller keeps publishing a "search" spin whenever it sees no line. At priority 20 that would block navigation (10) forever, so it only runs with `line_follow:=true`.

### What broke and how it was diagnosed
1. **The lidar saw a wall that is not there (a 2 mm modeling error).**
   - *Symptom:* `scan_overlay.py` puts each lidar return into world coordinates and checks it against the walls in the world file. 88% matched; 12% formed a straight line about 3.3 m in front of the robot where the world has nothing.
   - *Diagnosis:* a line at constant distance ahead of the robot is what a tilted scan plane hitting the floor looks like. If the lidar is 0.083 m up and the floor hit is 3.3 m ahead, the tilt is atan(0.083 / 3.3) = 0.025 rad. The v1 model puts the caster ball 2 mm lower than the wheels' contact point, 0.08 m behind them: atan(0.002 / 0.08) = 0.025 rad. The numbers matched exactly.
   - *Fix:* raise the caster 2 mm in the v2 copy. Result: IMU pitch at rest 0.0000 rad, 360 of 360 returns within 5 cm of a modeled surface (median 4 mm). v1 never noticed because a camera does not care about a 1.4 degree tilt.
   - *Why it matters:* SLAM would have drawn a phantom wall in the map, and Nav2 would have planned around it.
2. **The robot kept driving after every command stopped.**
   - *Symptom:* `mux_check.py` publishes inputs on a fixed schedule and watches what reaches the drive. In the final "all inputs stopped" phase, `cmd_vel` went silent but odometry still showed 0.20 m/s.
   - *Diagnosis:* the installed `twist_mux` header shows it only publishes when an input message arrives; it never publishes a zero when inputs time out. The Gazebo drive plugin has no command timeout, so it kept executing the last command.
   - *Fix:* a fourth, lowest-priority input that always publishes zero (`idle_stop.py`). When every real source has timed out, the zero wins and the robot stops. The same check now passes every phase.
   - *Interview angle:* this is why real robot base controllers have a command watchdog.
3. **Namespaced mux had no inputs.** With `namespace:=robot1`, the mux topics did not exist. A parameter file keyed `twist_mux:` only applies to the node at `/twist_mux`, not `/robot1/twist_mux`. The `/**:` wildcard key fixes it (Nav2's own param files do the same).
4. **Sensor rates looked low.** `ros2 topic hz` measures wall-clock time, but the simulator runs at about 0.74x real time in the container. Measured from the message timestamps (simulation time), the lidar is at 9.9 Hz and the IMU at 49 Hz, matching the configured 10 and 50.

### How each gate item was measured
| Gate item | How | Result |
|---|---|---|
| Lidar and IMU rates | message header stamps over 10 s (sim time) | scan 9.9 Hz (configured 10), IMU 49 Hz (configured 50) |
| Scan matches the walls | `scripts/scan_overlay.py` at the spawn pose; RViz screenshot | 360/360 returns within 5 cm, median 4 mm; images in `docs/images/` |
| One parent per frame | `view_frames` and `ros2 topic info -v /tf` | every frame has one parent; `/tf` and `/tf_static` have one publisher each (`robot_state_publisher`) |
| No TF warnings | launch log | none; one inherited, harmless KDL inertia warning (DECISIONS #18) |
| Line input drives; teleop overrides | `scripts/mux_check.py` (root namespace and `/robot1`) | all 5 phases pass in both |
| v1 still works | `scripts/check_v1.sh` | passes |

---

## Gate 2: Ground truth, EKF, SLAM map, drift benchmark (2026-10-08)

### What was built
- **Ground truth** (`ground_truth/pose`, Gazebo p3d plugin): the robot's exact pose, used only to score estimates.
- **EKF** (`config/ekf.yaml`, robot_localization): fuses wheel speed and turn rate with the IMU gyro's turn rate; the only publisher of `odom -> base_link`. Runs in every mode from `nav_sim.launch.py`.
- **SLAM** (`launch/slam.launch.py`, slam_toolbox online async from the installed defaults) and the saved map `maps/nav_world.{yaml,pgm}`.
- **Benchmarks** (`benchmarks/`): `metrics.py` (tested), `route.py` + `drive_route.py` (scripted "teleop" that steers from ground truth), `odom_drift.py` (22.8 m loop, raw odometry vs EKF vs truth). **Gate tool** `scripts/map_check.py`.

### What broke and how it was diagnosed
1. **Raw odometry was 97 degrees off after two laps.** A spin test (command exactly one revolution, compare truth, wheel odometry, gyro) showed the robot turning 295 degrees while the wheels claimed 338. Two causes: the caster ball had full friction and was dragged sideways (fixed: near-frictionless, like a real ball caster), and, the bigger one, measured turn rates were a constant 0.86x the command at every speed while straight lines were exact. 0.16 / 0.86 = 0.186 = wheel separation + wheel width: Gazebo's cylinder contact touches the floor at the wheel rims. Setting the drive plugin's track width to the measured 0.186 m made turns exact. This is effective-wheelbase calibration, the same thing UMBmark does on real robots.
2. **No artificial noise was needed.** The plan said to add noise if odometry were nearly perfect. After calibration it still drifts tens of centimetres over 21.6 m from real (simulated) wheel slip when the chassis keeps turning as the wheels brake at the end of each turn.
3. **The first map missed the corner behind a box** (unknown cells). The exploration route was extended around it.
4. **Back-to-back simulator launches in one container failed** (the old gzserver had not released its port). Each launch now gets its own Gazebo port and teardown waits for gzserver to exit; the affected batch was rerun.

### Results (each traces to a file in `benchmarks/results/` or `docs/images/`)
| Drift run (commit 30524e4, 21.6 m loop) | Raw odometry mean / final error | Raw final heading | EKF mean / final error | EKF final heading |
|---|---|---|---|---|
| odom_drift_20261009_043135 | 0.502 / 0.527 m | +36.1 deg | 0.037 / 0.006 m | +2.0 deg |
| odom_drift_20261009_043429 | 0.568 / 0.661 m | +37.6 deg | 0.091 / 0.052 m | +3.6 deg |
| odom_drift_20261009_043714 | 0.658 / 0.629 m | +41.4 deg | 0.077 / 0.069 m | +2.6 deg |

- Map: 2608 of 2608 occupied cells within 0.1 m of a modeled wall or obstacle (median 0.015 m), `scripts/map_check.py`.
- TF during SLAM: `map -> odom` (slam_toolbox) `-> base_link` (EKF) `-> sensors` (robot_state_publisher), one parent per frame.
- Why the EKF wins: the gyro measures turn rate directly and is far more trusted (variance 4e-8) than wheel odometry's turn rate (1e-3), so heading stays right and position follows.

---

## Gate 3: Nav2 on the saved map (2026-10-09)

### What was built
- **`nav2.launch.py`**: the simulation plus map_server and AMCL (localization on the saved map; AMCL owns `map -> odom`) and the Nav2 servers (planner, controller, behavior tree navigator, recoveries, velocity smoother), all as components in one process.
- **`nav2_params.yaml`**: the installed Humble defaults with every change marked and explained in `docs/TUNING.md`.
- **Goal benchmark** (`benchmarks/nav_goals.py`, `goals.yaml`): 11 goals through every room and doorway, each scored against ground truth (success, time, distance driven, final error, recoveries, AMCL error).
- **Checks**: `mux_check.py --nav-goal` (teleop overrides Nav2, Nav2 resumes), a goal sent the way RViz's 2D Goal Pose tool sends it, and a recorded RViz GIF.

### Why these choices
- **Every Nav2 velocity goes through twist_mux.** The stock bringup sends the smoothed controller output, and the recovery behaviors (spin, back up), straight to the drive topic. That would let Nav2 bypass the priority scheme, so a human at the keyboard could not take over during a recovery. Here everything Nav2 sends enters the mux on `cmd_vel_nav`.
- **Tune by measurement, one round at a time.** Start from the installed defaults, run the 11-goal benchmark, read the log for why things went wrong, change only what the evidence points at, rerun.

### What broke and how it was diagnosed
1. **35 recoveries on the defaults, all CPU starvation.** Nothing was wrong with the plans; the log showed the behavior tree giving up after waiting the default 20 ms for the controller to acknowledge a path (24 times), and 727 missed controller cycles. Container CPU is shared with Gazebo. Longer acknowledgement timeout, 10 Hz controller, 2D obstacle layer instead of a 3D voxel grid (the lidar is 2D), debug output off: recoveries fell to 16, and the goal tolerance went from 0.25 m (longer than the robot) to 0.15 m.
2. **One goal kept failing at a doorway (DWB local minimum).** `g08_west` leaves a room through a doorway and has to U-turn around the end of the room's wall. Recording the robot's true path and every velocity command showed it sat in the doorway for 83 s, 76% of the time turning in place with the direction flipping 69 times. DWB, the default controller, scores sampled trajectories with several critics; the "go toward the goal" critics were pulling toward the end of the path it could see (which lies past the wall end) while the "follow the path" critics pulled along the arc. They cancelled. Switching to Regulated Pure Pursuit, which just chases a point a short distance ahead on the path and turns in place first when that point is far off its heading, fixed it: 11 of 11 goals, zero recoveries, and that leg went from aborting after 171 s to 32 s.
3. **A launch argument leaked into Gazebo.** Naming the Nav2 parameter file argument `params_file` also handed it to Gazebo, whose launch file has an argument of the same name; launch arguments are visible to included launch files. Renamed to `nav2_params`.
4. **A one-in-twelve startup hang.** Nav2's lifecycle manager once waited forever for the smoother server to answer "configure". It never recurred in 8 deliberate restarts, so the cause is not proven (likely a lost service reply between processes). Running Nav2 as components in one process, the stock Humble default, removes those cross-process calls and also cut startup to 10 to 23 s.
5. **My own measurement mistakes, caught and fixed.** The benchmark's AMCL-error column compared AMCL's last published estimate with where the robot is now, but AMCL only publishes after it has moved 0.25 m; it now compares at the estimate's timestamp. And the first teleop-over-Nav2 check sent its goal before Nav2's subscription was connected, so the goal was silently dropped; the check now waits for the connection.

### Results (each traces to `benchmarks/results/`)
| Run | Success | Mean time | Mean final error | Recoveries |
|---|---|---|---|---|
| Baseline (installed defaults, DWB) | 11/11 | 27.2 s | 0.182 m | 35 |
| Round 1 (CPU settings, 0.15 m tolerance) | 10/11 | 22.6 s | 0.110 m | 16 |
| Round 2 (composed, Regulated Pure Pursuit) | 11/11 | 21.0 s | 0.100 m | 0 |

Final error is measured with ground truth, so it includes AMCL's localization error (0.02 to 0.17 m at goal ends) on top of the 0.15 m goal tolerance: Nav2 decides it has arrived using its own estimate of where it is.

### Gate checks
| Gate item | How | Result |
|---|---|---|
| A goal sent from RViz is reached | published on `goal_pose` (the message RViz's 2D Goal Pose tool sends), RViz recorded | "Goal succeeded"; `docs/images/gate3_rviz_goal_reached.png`, `docs/images/gate3_nav2_run.gif` |
| 10+ goals benchmarked, real success rate recorded | `benchmarks/nav_goals.py`, 11 goals | 11/11, 0 recoveries (round 2); all three rounds kept in `benchmarks/results/` |
| Teleop still overrides Nav2 | `scripts/mux_check.py --nav-goal -3.0 -0.5` | Nav2 driving (turning at -1.0 rad/s), then teleop's +0.8 rad/s reached the drive exactly (the opposite direction), then Nav2 resumed and finished the goal 0.046 m from it |
| v1 still works | `scripts/check_v1.sh` | passes |

---

## Gate 4: RobotAvoidanceLayer, a custom C++ Nav2 costmap plugin (2026-10-10)

### The problem it solves (measured, not assumed)
The robots cannot see each other. The lidar scans a plane 0.083 m above the floor; the chassis top is at 0.063 m, and the lidar housing has no collision shape in the simulation. A box with this robot's footprint placed 0.9 m straight ahead of the lidar is invisible: the scan read 4.95 m, the wall behind it. Nav2's obstacle layer only marks what the lidar sees, so without help a robot will drive straight into another one.

### What was built
- **`vision_bot_plugins`** (C++17, ament_cmake): `RobotAvoidanceLayer`, a `nav2_costmap_2d::Layer` loaded by Nav2 through pluginlib. It subscribes to each other robot's pose topic (the type AMCL publishes), keeps the latest pose per robot under a mutex (subscriber callbacks and the costmap update run on different threads), skips poses older than 2 s (a robot that goes offline stops blocking paths), transforms each pose into the costmap's frame, and writes a lethal disk of the other robot's radius plus a cost that decays with distance.
- **Pure cost math in its own library**, unit tested with gtest (8 tests): lethal (254) inside the radius, then `floor(252 * exp(-k * (d - r)))`, which starts at 252 so only the robot's own disk counts as certain collision (253 and above), never rises with distance, and reaches 0 at a computed influence radius used for the layer's update bounds.
- **A launch test** that starts Nav2's standalone costmap with only this layer and checks it initializes and activates without errors.
- **An A/B benchmark** (`benchmarks/plugin_ab.py`) with a second robot (a box with this robot's footprint, moved by a Gazebo plugin) in two scenarios, the layer off and on, same seeds.

### How a costmap layer works (worth being able to explain)
Each costmap cycle, the layered costmap asks every layer for the area it wants to change (`updateBounds`), resets the master grid inside the union of those areas, then asks each layer in order to write its costs (`updateCosts`). Two consequences shaped the code: the layer reports last cycle's area as well as this cycle's, or a robot's old position would never be erased after it moves; and it is listed before the inflation layer, so Nav2 inflates the other robot's disk by this robot's own size like any other obstacle.

### What broke and how it was diagnosed
1. **"Succeeded" while 0.8 m away.** In the first smoke run without the layer, the robot shoved past the invisible robot, the collision corrupted its localization, and Nav2 reported success while ground truth put it 0.80 m from the goal. The benchmark now reports "truly arrived" (Nav2 success and within 0.30 m by ground truth) next to Nav2's own success.
2. **The launch test could not find the costmap.** Nav2's standalone costmap executable creates its node as `/costmap/costmap`, not `/costmap`, and does not create the "bond" heartbeat the lifecycle manager expects from Nav2 servers; the test names it correctly and turns bond checking off.

### Results (`benchmarks/results/plugin_ab_20261010_065758.md`, 10 trials per scenario per condition)
| Scenario | Layer | Nav2 success | Truly arrived | Recoveries | Mean min separation | Contacts |
|---|---|---|---|---|---|---|
| Parked in the doorway | off | 10/10 | 0/10 | 0 | 0.195 m | 10/10 |
| Parked in the doorway | on | 10/10 | 10/10 | 0 | 0.438 m | 0/10 |
| Crossing, robot2 does not yield | off | 10/10 | 10/10 | 0 | 0.239 m | 4/10 |
| Crossing, robot2 does not yield | on | 6/10 | 5/10 | 133 | 0.257 m | 6/10 |

- **Parked robot: the layer fixes the problem completely.** Without it, every run hit the invisible robot and none truly arrived; with it, no contacts and every run arrived, at a cost of 1.3 s.
- **Crossing robot: the layer makes things worse, and why.** The layer draws where the other robot *is*, not where it is *going*. The robot drives toward the crossing point because the other robot is not in its way yet, sees it at the last moment, and stops in its path; the scripted robot2 never yields and hits it, and while the other robot's disk overlaps the robot's own position the planner cannot plan, hence the recoveries. Without the layer the robot simply keeps going and usually clears the crossing first. The fix is motion prediction (drawing the other robot's path a few seconds ahead); it was not added because it changes the layer's design and needs Zach's decision. In Task 5 both robots run the layer, so neither drives blind.

## Gate 5: Two robots on one map (2026-10-12)

### The problem
One robot running Nav2 is a solved setup; two copies of it in one ROS graph collide on every name. Both would publish `cmd_vel`, `scan` and `odom`, both would broadcast `odom -> base_link` into the same `/tf`, and both lifecycle managers would try to manage the same `/amcl`. On top of that, Gate 4 showed the robots cannot see each other with their lidar, so each needs the other's position from somewhere else.

### What was built
- **`multi_robot.launch.py`**: one Gazebo world, two complete stacks under `/robot1` and `/robot2`, each with its own robot_state_publisher, EKF, velocity mux, AMCL, map server and Nav2, and its own TF tree, both loading the same saved map. It follows the multi-robot example launch that ships with Nav2.
- **Per-robot parameters from one YAML file** (`robot_params.py`, five unit tests): namespace, AMCL's initial pose set to the spawn pose, and each robot's avoidance layer pointed at the other robot. One file means the two robots cannot drift apart in tuning.
- **`shared_pose`**: each robot publishes its map pose at 10 Hz for the other robot's avoidance layer.
- **Namespaced benchmarks**: `nav_goals.py --namespace robotN`, and `plugin_ab.py --robot1 robot1 --robot2 robot2`, which reruns the Gate 4 A/B with the real second robot driving under its own Nav2.

### Worth being able to explain
- **Why TF is remapped per robot.** TF is one global topic by default. Two robots using the same frame names (`odom`, `base_link`) would publish contradictory transforms into one tree. Remapping each robot's `/tf` to `/robotN/tf` gives each robot a private tree with the same frame names, so the Nav2 configuration needs no frame prefixes.
- **Why not AMCL's own pose topic.** AMCL publishes its pose only after a filter update, which needs the robot to move: with both robots parked, `amcl_pose` sent 0 messages in 8.7 s. The other robot's layer drops poses older than 2 s, so a parked robot would vanish from its costmap exactly when it is a static obstacle. A fixed-rate relay fixes that.

### What broke and how it was diagnosed
1. **The PC went to sleep mid-run.** Nav2's servers send a heartbeat ("bond") every 0.1 s and the lifecycle manager shuts the stack down after 4 s without one; after the host woke, both stacks reset and the benchmark recorded 11 of 11 goals "rejected". Found from a 16-hour gap in the log timestamps. Since then every run holds the machine awake and every script prints "Nav2 lifecycle failures (must be 0)". It happened once more when an app restart dropped the hold; the log timeline showed trials 1 to 3 finished 14 s before the gap, which separated the valid trials from the invalid one. Invalid runs are kept out of `benchmarks/results`.
2. **A refactor broke the A/B on its first goal.** A shared helper wrote `msg.pose.pose`, which exists on the initial-pose message but not on a Nav2 goal. The one-trial smoke test caught it before any real run; fixed with a module-level function and a test for each message type.
3. **`ros2 node list` undercounted.** One listing showed robot1 with 5 nodes while both stacks were running; the command reports whatever the discovery daemon has seen so far. Retrying until it settled showed 26 named nodes in each namespace, identical sets, and only Gazebo and the launch process outside them.
4. **A metric calibrated on a box.** The contact count (centres closer than 0.20 m) shows 1/10 for the parked robot without the layer, but robot1 ended pressed against robot2 in all 10 runs, 1.45 to 1.66 m short of its goal. Two real chassis touch at anywhere from under 0.20 m (side on) to 0.24 m (corner to corner), so a centre distance cannot decide contact. The ground-truth "truly arrived" column is the outcome measure there; the crossing contact counts are unaffected.

### Results
**Each robot drives the 11-goal tour while the other is parked** (`nav_goals_robot1_20261011_234057.md`, `nav_goals_robot2_20261011_235724.md`):

| Run | Success | Mean time | Mean final error | Recoveries | Closest approach | Parked robot moved |
|---|---|---|---|---|---|---|
| One robot alone (Gate 3, round 2) | 11/11 | 21.0 s | 0.100 m | 0 | n/a | n/a |
| robot1 drives, robot2 parked | 11/11 | 21.3 s | 0.119 m | 0 | 0.81 m | 0.007 m |
| robot2 drives, robot1 parked | 11/11 | 22.5 s | 0.102 m | 0 | 0.95 m | 0.007 m |

The largest change from the single-robot run is g08_west (32.0 s to 35.7 s, path 6.92 m to 7.45 m), whose goal is 0.7 m from robot2's parking spot. Isolation: a 3 s turn command on `/robot1/cmd_vel_teleop` turned robot1 159 degrees while robot2 turned 0.0 degrees and moved 0.0002 m.

**RobotAvoidanceLayer A/B with two real robots** (`plugin_ab_2robots_20261013_030935.md`, 10 trials per scenario per condition, the layer switched on both robots together):

| Scenario | Layer | Nav2 success | Truly arrived | Mean time | Recoveries | Worst min separation | Contacts | robot2 reached its goal |
|---|---|---|---|---|---|---|---|---|
| Parked in the doorway | off | 10/10 | 0/10 | 14.0 s | 0 | 0.198 m | 1/10 (really 10/10, see above) | n/a |
| Parked in the doorway | on | 10/10 | 10/10 | 16.7 s | 0 | 0.386 m | 0/10 | n/a |
| Crossing | off | 10/10 | 10/10 | 18.7 s | 0 | 0.190 m | 2/10 | 10/10 |
| Crossing | on | 9/10 | 9/10 | 22.7 s | 29 | 0.269 m | 0/10 | 9/10 |

- **Parked robot: same as Gate 4.** Without the layer the robot drives into the robot it cannot see every time; with it, every run arrives.
- **Crossing: the Gate 4 result reverses.** Against a scripted robot that never yielded, the layer made things worse. With both robots running it, each slows for the other, and contacts go from 2/10 to 0/10, at a cost of 4 s per crossing.
- **The remaining failure is a symmetric standoff.** In one run both robots saw each other at the same instant, ran the same recovery sequence in step (wait, spin, back up), and every spin and back-up failed at once because the other robot's disk was in the way; neither moved and both gave up 26 s later. Breaking it needs asymmetry: a right-of-way rule, or predicting the other robot's motion so one yields early. That is a design decision left to Zach.

## Gate 6: Connecting the robots to the fleet platform (2026-10-14)

### The problem
The fleet platform (a separate FastAPI + Postgres project) tracked robots, telemetry and maintenance, but had no way to send a robot somewhere and no notion of a position on a map. The simulated robots had Nav2 but no connection to the fleet. The job: a bridge per robot that keeps the fleet's picture of the robot current and turns goals queued in the fleet into Nav2 goals, and that survives everything a network service does to a client: expired tokens, restarts, outages, slowness.

### What was built
- **Fleet API additions** (fleet repo, branch `v2/goals-api`, approved at Gate 0): a `goals` table with `POST /robots/{id}/goals`, `GET /robots/{id}/goals?status=`, `PATCH /goals/{id}` for admins and operators; a finished goal cannot change and status never returns to pending; every change is audited. Telemetry gained a map-frame pose, a navigation state and a `battery_simulated` flag. One Alembic migration with a working downgrade; 15 new tests, passing on SQLite and Postgres.
- **`fleet_bridge`** (ROS 2 package): an HTTP client (login, refresh once on 401 and store the rotated refresh token, log in again if the refresh is refused, never log a token), a goal manager (one active goal per robot; a new goal replaces the old one), the bridge's timing and recovery logic kept free of ROS so it is unit tested (registration with exponential backoff, status reports queued in order through outages, telemetry every second with a battery that drains with distance), and a thin ROS node. 54 tests.
- **Least privilege**: the bridge runs as an operator account, which can post telemetry and update goals but cannot create or delete robots or users. An admin seed script creates the robots and the bridge account once; the generated password goes only to a git-ignored file.
- **A disposable test stack** of the fleet API on its own port and database, so none of the checks touched the stack in daily use.

### Worth being able to explain
- **Why HTTP runs on its own thread.** An HTTP call can block for seconds; on ROS's single-threaded executor that would stall the Nav2 action callbacks. The fleet thread only queues goal sends and cancels, and a 10 Hz ROS timer carries them out, so every ROS call stays on ROS's own thread.
- **Why "accepted" is reported when Nav2 accepts, not when the bridge sends.** Until then the goal is still pending in the fleet, so the next poll returns it again; the goal manager recognizes the repeat and ignores it.
- **Rate limiting is a deployment setting.** The API allows 100 requests a minute per client address; two robots each posting telemetry every second from one machine exceed that, so a fleet serving bridged robots needs a higher limit, and the bridge treats a 429 as temporary.

### What broke and how it was diagnosed
1. **A log-capture test saw nothing.** ROS's pytest plugin installs its own logger class whose loggers do not pass records up to the root logger, where pytest listens. Found by printing the logger's state inside a test (`propagate=False`) and tracing the class to `launch.logging`. The test now attaches its handler to the client's logger directly, and a deliberately planted token leak confirmed the test catches one.
2. **A replacement goal failed within a second.** The bridge cancelled the old goal and sent the new one back to back; Nav2's action server, on a cancel, terminates every goal it holds, including one that has arrived but not started. Found by lining up the bridge log with Nav2's (no "Begin navigating" for the new goal). Fixed by holding a new goal until the old one has finished in Nav2; the rerun shows the new goal sent 5 ms after the cancellation completed, and it succeeded.
3. **Requests stalled for seconds every few minutes.** Probing the API from inside its container and from the host at the same time showed most stalls were in Docker Desktop's port forwarding under simulation load, not in the API. The bridge rode them out; the check script had to be taught to do the same, and the timings below come from the API's own records.
4. **A status report rejected with "too many requests" would have been dropped** as if permanent; spotted while reading the rate limiter, fixed test-first before any real run.

### Results (fleet test stack, 1-minute access tokens, both robots running)
| Check | Result |
|---|---|
| Register and post telemetry, both robots | Both resolved by serial number as the operator account; about one reading per second with pose, navigation state and simulated battery |
| Reachable goal from the fleet, each robot | pending, accepted, succeeded (robot1 20 s, robot2 23 s) |
| Unreachable goal (off the map) | accepted, then `failed` when Nav2 gave up after 23 s |
| New goal while one is driving | old goal `canceled`, new goal `succeeded` |
| API killed for 30 s, then restarted | bridges kept running; telemetry gap 54 s (30 s down plus API startup); a goal sent afterwards succeeded |
| Token expiry | 8 routine refreshes, none failed |
| Signing key rotated (all tokens invalid) | refresh refused, logged in again, telemetry gap 25 s |
| Nav2 lifecycle failures, bridge crashes, rate-limit rejections | 0, 0, 0 |

## Gate 7: A map view with click-to-goal (2026-10-15)

### The problem
The dashboard showed one robot's camera and line-following state through rosbridge. With a fleet platform in place, an operator should be able to see every robot on the map and send one somewhere by clicking, with the fleet platform, not a direct ROS connection, as the single path for commands, so every goal is authenticated, recorded and visible to the rest of the fleet.

### What was built
- **Map math**: conversions between image pixels and the map frame (ROS map conventions: the origin is the image's bottom-left corner and image rows count down), with tests. The saved map is converted to a PNG by a small standard-library script, since browsers cannot show PGM.
- **Server routes in the dashboard** that log into the fleet API with their own operator account, so no token reaches the browser: list robots with their latest telemetry, queue a goal, and read a goal's status.
- **The map view**: the map with each robot's position, heading and state (green idle, blue navigating, grey when telemetry is more than 5 s old), a click that sends a goal to a chosen robot or the nearest idle one, and a list of goals with live status, `failed` in red.
- **A test runner** for the dashboard (vitest, 29 tests).

### Worth being able to explain
- **Why the SVG overlay uses the map's own pixel grid.** The overlay's coordinate system is the image's 200 x 200 pixels, so robot poses and clicks convert through one pure function regardless of how large the map is drawn; the browser's screen transform handles the rest.
- **Why goals go through the fleet API instead of rosbridge.** One path for commands means one place for authentication, auditing and status; the robots' bridges are the only things that talk to Nav2.

### What broke and how it was diagnosed
1. **A click failed with "the fleet API is not answering."** Server-side metrics showed the API fast; probing showed the failures only after a pause: the API closes idle connections after 5 s and Node reused one just as it closed (6 of 24 requests after 5 s gaps, none of 445 without gaps). Fixed with one retry on a reset connection, test-first.
2. **Requests hung for up to a minute.** The API answered in milliseconds from inside its container, so the delay was outside it; the host had 570 MB of free memory and 4 GB paged out, and the same stalls then took down the Nav2 stacks (8 lifecycle failures), so that run was discarded. A 10 s client timeout keeps the map responsive, and the rerun used the lighter production build of the dashboard (0 failures).
3. **Nav2 said "succeeded" for a goal inside an obstacle.** Its planner moves an unreachable goal to the nearest free cell and the controller checks arrival against that, so the robot stopped 0.48 m from the clicked point. The bridge now checks the robot's own position against the requested goal and reports `failed` beyond 0.30 m, the same "truly arrived" threshold as the Gate 4 benchmark.
4. **My first scripted clicks landed in the wrong place** because screenshots are scaled relative to the page. The checks then clicked through the page itself, converting a world point to screen coordinates from the map element's own position.

### Results
| Check | Result |
|---|---|
| Click on the east room, "nearest idle robot" | goal 15 to robot1, accepted, robot drives through the doorway, `succeeded` (`docs/images/gate7_click_to_goal.gif`) |
| Click inside an obstacle (box centre) | goal 18: Nav2 reported success 0.48 m away; the bridge reported `failed`, shown in red |
| Click on a thin wall | robot stops against the wall about 0.30 m from the point; reported `succeeded` (see DECISIONS #71) |
| Camera stream, telemetry panel, manual/auto toggle, reconnect | stream live (about 15 frames/s), detection values update, the toggle published `false` on `/vision_bot/autonomous_enabled`, and after rosbridge was stopped and restarted the dashboard reconnected on its own in 10 s |
| Nav2 lifecycle failures in the valid run | 0 |
