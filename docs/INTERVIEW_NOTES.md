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
