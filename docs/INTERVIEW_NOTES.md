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
