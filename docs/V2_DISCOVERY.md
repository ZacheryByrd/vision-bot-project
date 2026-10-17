# vision_bot v2: Discovery (Task 0)

Date: 2026-10-08. Two passes:
- **Static pass:** both repos read from the cloud workspace.
- **Live pass:** run on Zach's PC in disposable `vision_bot_dev:humble` containers (`docker compose run --rm`), v1 sim headless under `xvfb-run`, fleet API queried at `http://localhost:8000`. Every fact below marked **(live)** was observed, not inferred. The live pass corrected one static finding (odometry, see "Findings that change the plan" #1).

## Environment (live, runbook section 1.1)
```
$ printenv ROS_DISTRO
humble
$ ls /opt/ros
humble
$ lsb_release -a
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.5 LTS
Release:        22.04
Codename:       jammy
$ gz sim --version 2>/dev/null || ign gazebo --version 2>/dev/null || gazebo --version 2>/dev/null
Gazebo Sim, version 6.18.0
$ gazebo --version
Gazebo multi-robot simulator, version 11.10.2
$ colcon --help >/dev/null && echo "colcon ok"
colcon ok
$ python3 --version
Python 3.10.12
```
- The runbook's one-line version check stops at the first match, and `osrf/ros:humble-desktop-full` ships **both** Gazebo Sim 6 (Fortress, `ros-humble-ros-gz-*`) and Gazebo Classic 11.10.2. The installed set alone does not decide the stack; the repo does (next section).
- Dev environment is Docker on Windows (`docker/docker-compose.yml`): service `ros`, image `vision_bot_dev:humble`, `../ros2_ws` bind-mounted at `/workspace/ros2_ws`, and (added in Task 0) the whole repo at `/workspace/vision-bot-project` so `scripts/` and `benchmarks/` run in the container. GUI via VcXsrv (`DISPLAY=host.docker.internal:0.0`, `LIBGL_ALWAYS_SOFTWARE=1`); `xvfb` for headless runs.
- Host ports: rosbridge 9091->9090, web_video_server 8081->8080.
- The host has no ros2/colcon; all sim, colcon, and gate checks run in the container.

## Simulator stack: Gazebo Classic 11 (decided, plan Task 0 step 2)
Evidence from the repo: `sim_launch.py` includes `gazebo_ros/launch/gazebo.launch.py` and spawns with `gazebo_ros/spawn_entity.py`; the URDF uses `libgazebo_ros_diff_drive.so` and `libgazebo_ros_camera.so`; worlds are `.world` files. Classic 11 is installed and supported on Humble, so no migration question arises. **Use the [Classic] variant of every plan step.**

## v1 inventory
- Workspace: `ros2_ws/` (build/install/log git-ignored). One package: `ros2_ws/src/vision_bot` (ament_python).
- Nodes: `perception_node`, `motor_control_node`, `gpio_motor_driver` (hardware only).
- Launch files: `sim_launch.py` (base), `line_follow_launch.py`, `dnn_follow_launch.py` (reuse sim_launch with args), `bridge_launch.py` (rosbridge + web_video_server), `hardware_launch.py`.
- Worlds: `track.world`, `line_track.world`, `person_track.world.in`.

### Live graph: `ros2 launch vision_bot sim_launch.py gui:=false` (track.world)
`ros2 node list` (live):
```
/camera_controller
/gazebo
/motor_control_node
/perception_node
/robot_state_publisher
```
(The diff-drive plugin also appears as node `diff_drive` in `ros2 topic info -v /cmd_vel`.)

`ros2 topic list -t` (live):
```
/camera/camera_info [sensor_msgs/msg/CameraInfo]
/camera/image_raw [sensor_msgs/msg/Image]
/camera/image_raw/compressed [sensor_msgs/msg/CompressedImage]
/camera/image_raw/compressedDepth [sensor_msgs/msg/CompressedImage]
/camera/image_raw/theora [theora_image_transport/msg/Packet]
/clock [rosgraph_msgs/msg/Clock]
/cmd_vel [geometry_msgs/msg/Twist]
/joint_states [sensor_msgs/msg/JointState]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/performance_metrics [gazebo_msgs/msg/PerformanceMetrics]
/robot_description [std_msgs/msg/String]
/rosout [rcl_interfaces/msg/Log]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
/vision_bot/autonomous_enabled [std_msgs/msg/Bool]
/vision_bot/debug_image [sensor_msgs/msg/Image]
/vision_bot/detection [std_msgs/msg/Float32MultiArray]
```
- `/cmd_vel` (live, `ros2 topic info -v`): 1 publisher `motor_control_node`, 1 subscriber `diff_drive`, type `geometry_msgs/msg/Twist`. The name is absolute in code but remappable from a launch file.
- `/vision_bot/detection` is Float32MultiArray `[detected, offset_x, offset_y, area_fraction, confidence]` (from code).
- `/camera/image_raw`: configured 30 Hz in the URDF; **measured 16.4 to 17.4 Hz** headless under xvfb with software GL (`ros2 topic hz`, window 54 to 75 msgs).
- **No `/odom` topic.** `ros2 topic info /odom` -> `Unknown topic`.

### Live TF
- `/tf` publishers: 1 (`robot_state_publisher`); no messages observed on `/tf` in 5 s (no `/joint_states` publisher, so the wheel joints are never broadcast).
- `/tf_static` publishers: 1 (`robot_state_publisher`): `base_link -> camera_link` (0.10, 0, 0.03) and `base_link -> caster_wheel` (-0.08, 0, -0.02).
- `view_frames` (live):
```
"base_link" -> "camera_link"   (static)
"base_link" -> "caster_wheel"  (static)
```
- `tf2_echo odom base_link`: `Invalid frame ID "odom" ... frame does not exist`.
- **Who publishes `odom -> base_link` today: nobody.** `left_wheel`/`right_wheel` are also absent from TF.
- Launch log: one pre-existing warning, `kdl_parser: The root link base_link has an inertia specified in the URDF, but KDL does not support a root link with an inertia` (harmless for v1; a `base_footprint` root in the v2 xacro removes it). Remaining stderr is ALSA noise from the headless container.

### URDF (`description/vision_bot.urdf.xacro`)
- Links: `base_link`, `left_wheel`, `right_wheel`, `caster_wheel`, `camera_link`.
- Chassis 0.20 x 0.14 x 0.06 m; wheel radius 0.033 m; wheel separation 0.16 m; caster sphere radius 0.015 m; camera at (0.10, 0, 0.03) from `base_link`.
- Footprint for Nav2: the chassis rectangle is 0.20 x 0.14 m, so the circumscribed radius is about 0.122 m (sqrt(0.10^2 + 0.07^2)).
- Diff-drive plugin block sets only `left_joint`, `right_joint`, `wheel_separation`, `wheel_diameter`, `command_topic`, `odometry_topic`, `robot_base_frame`. It does **not** set `publish_odom`, `publish_odom_tf`, or `publish_wheel_tf`; in ROS 2 `gazebo_plugins` these default to false (the installed header's usage example sets them to true explicitly). That is why there is no `/odom` and no odom TF (live evidence above).

### Existing tests and CI
`test/test_motor_control_logic.py`, `test/test_perception_logic.py` (pure functions, no colcon needed). CI (`.github/workflows/ci.yml`): `ros` job (pytest + flake8 in `osrf/ros:humble-desktop`) and `dashboard` job (`npm ci && npm run build`).

## Findings that change the plan
1. **v1 has no odometry at all (live; corrects the static pass).** The static pass assumed the diff-drive plugin publishes `odom` and the `odom -> base_link` TF and that the TF had to be switched off. In fact neither is published. Task 1's xacro must set `publish_odom: true` and `publish_odom_tf: false` (the EKF will own the TF from Task 2), and either `publish_wheel_tf: true` or a `gazebo_ros_joint_state_publisher` so the wheel frames exist. Until the EKF runs in Task 2, nothing publishes `odom -> base_link`, so Task 1's RViz check uses a fixed frame of `base_link` (or a temporary static transform; decided in Task 1).
2. **v1 will fight Nav2 in twist_mux.** `motor_control_node`'s watchdog publishes a Twist (a search spin when `search_when_lost` is true, otherwise zeros) every cycle once no target has been seen for `lost_timeout_sec` (0.75 s). On `cmd_vel_line` (priority 20 > nav 10) that blocks Nav2 permanently. Task 1 ruling (DECISIONS #4): `nav_sim.launch.py` starts `perception_node`/`motor_control_node` only when `line_follow:=true` (default false); `remappings=[("/cmd_vel", "cmd_vel_line")]`; no v1 code change.
3. **Twist, not TwistStamped**, on Humble: `/cmd_vel` is `geometry_msgs/msg/Twist` (live) and twist_mux 4.3.0 (Humble) uses Twist.
4. **Docker is required for every sim step.** Gate commands run in the container.

## Dashboard (vision-bot-project/dashboard)
- Next.js app router (`app/`, no `src/`), `roslib` in the browser (`app/hooks/useRosConnection.ts`). Code defaults `ws://localhost:9090` and `http://localhost:8080/stream?topic=/vision_bot/debug_image`; `.env.local.example` sets `NEXT_PUBLIC_ROS_WS_URL=ws://localhost:9091` and `NEXT_PUBLIC_VIDEO_STREAM_URL=http://localhost:8081/stream?topic=/vision_bot/debug_image` (the Docker host ports).
- Subscribes: `/vision_bot/detection`, `/cmd_vel`. Publishes: `/vision_bot/autonomous_enabled` (Bool) for the manual/autonomous toggle.
- Components: ControlPanel, Header, StatusPanel, VideoFeed. No fleet-API or auth code. `package.json` has `dev`, `build`, `lint`, and **no `test` script** (Task 7 must add a test runner).

## Fleet platform
- **Local repo:** `C:\Users\zache\robot-fleet-management` (GitHub `robot-fleet-managment-system`), branch `main` at `b200721`, clean working tree.
- **Running locally (live):** compose project `robot-fleet-management` (`-api-1`, `-frontend-1`, `-postgres-1`), API at `http://localhost:8000`, OpenAPI at `/openapi.json` (title "Robot Fleet Management" 0.1.0). Endpoints below are from the live OpenAPI plus the route code for roles.
- FastAPI + SQLAlchemy + Alembic + PostgreSQL; Next.js BFF frontend; base path `/api/v1`.

| Method + path | Request | Response | Roles |
|---|---|---|---|
| `POST /api/v1/auth/login` | form (`x-www-form-urlencoded`): `username` (**the user's email**), `password` | `Token {access_token, refresh_token, token_type}` | public |
| `POST /api/v1/auth/refresh` | JSON `{refresh_token}` | new `Token` pair | public (valid refresh token) |
| `GET /api/v1/auth/me` | - | `UserRead {id, email, full_name, is_active, role {id, name}, created_at}` | any authenticated |
| `GET /api/v1/robots?skip&limit` | - | `Page {items: RobotRead[], total, skip, limit}`; **no filter by serial** | admin, operator |
| `POST /api/v1/robots` | `RobotCreate {name, serial_number, model, location?, status?}` | 201 `RobotRead` | **admin only** |
| `GET /api/v1/robots/{id}` | - | `RobotRead {id, name, serial_number, model, location, status, last_seen_at, created_at, updated_at}` | admin, operator |
| `PATCH /api/v1/robots/{id}` | `RobotUpdate {name?, model?, status?, location?}` | `RobotRead` | admin, operator |
| `DELETE /api/v1/robots/{id}` | - | 204 | admin only |
| `GET /api/v1/robots/status-counts` | - | `{status: count}` | admin, operator |
| `POST /api/v1/telemetry/robots/{id}` | `TelemetryReadingCreate {battery_level?, temperature_celsius?, latitude?, longitude?, speed_mps?, recorded_at?}` (all nullable) | 201 `TelemetryReadingRead` | admin, operator |
| `GET /api/v1/telemetry/robots/{id}?skip&limit` | - | `TelemetryReadingRead[]` | admin, operator |

- `RobotStatus` enum: `idle | active | charging | maintenance | offline`. Robot `id` is an integer; `serial_number` is unique.
- Tokens (`app/core/config.py`): `ACCESS_TOKEN_EXPIRE_MINUTES=60`, `REFRESH_TOKEN_EXPIRE_DAYS=7`, both settings (env-overridable), so Task 6 can force expiry by running the API with a short access lifetime.
- Refresh **rotates** (returns a new refresh token) but has **no server-side revocation**: the old refresh token keeps working until it expires (`auth.py` comment). The bridge must still store the rotated token (plan test `test_rotated_refresh_token_is_stored`); the real API will not punish reuse.
- **Missing for v2:** no goals/missions endpoints, no per-robot navigation state, no map-frame pose fields. The additions below were approved (DECISIONS #2).

## Approved fleet additions (DECISIONS #2; Task 6)
1. New `goals` table (id, robot_id FK, x, y, yaw, status: pending|accepted|succeeded|failed|canceled, created_by, timestamps) via an Alembic migration.
2. `POST /robots/{id}/goals`, `GET /robots/{id}/goals?status=pending`, `PATCH /goals/{id}` (all admin/operator), with pytest tests following `tests/test_robots.py`.
3. Nullable `pose_x`, `pose_y`, `pose_yaw`, `nav_state` columns on telemetry readings.

## Open question for Zach (Task 6)
**Bridge service-account role.** `POST /robots` is admin-only, so a bridge that registers its own robot needs an admin account. Recommendation: the bridge runs as an **operator** account and only *resolves* its robot by `serial_number` (paging `GET /robots`); the two sim robots are created once by an admin seed script (`scripts/`), so a leaked bridge credential cannot create or delete robots. Alternative: give the bridge admin and let it create on first start.

## Resolved design gap
Dashboard-to-fleet auth: DECISIONS #3 (a Next.js server route in the vision_bot dashboard logs in with a service account from env vars; tokens never reach the browser).

## Baseline (Task 0 step 7, live, rebuilt image `vision_bot_dev:humble` 77022f35928c)
The rebuild re-pulled a newer `osrf/ros:humble-desktop-full` base (for example `gazebo-plugins` 3.9.0 build 20260907), so this baseline is on the refreshed base plus the v2 layer.

| Check | Command (in the container) | Result |
|---|---|---|
| Clean build | `colcon build` into an empty temp base | `Summary: 1 package finished [5.34s]`; only stderr is a pre-existing setuptools `UserWarning` about dash-separated `script-dir` in `setup.cfg` (not an error) |
| v1 unit tests | `python3 -m pytest test/ -q` in `ros2_ws/src/vision_bot` | `23 passed in 6.54s` |
| v1 lint (CI settings) | `python3 -m flake8 --max-line-length=100 vision_bot/ test/` | clean (no output) |
| v1 regression check | `scripts/check_v1.sh` | exit 0: `nodes up: perception_node motor_control_node`, `/camera/image_raw publishing at 14.431 Hz`, `PASS` |
| Check can fail | `V1_NODES="perception_node motor_control_node no_such_node" V1_TIMEOUT_S=45 scripts/check_v1.sh` | exit 1: `FAIL: nodes not up after 45s: no_such_node` |

Camera rate under headless software rendering varied from 14.4 to 17.4 Hz across runs (configured 30 Hz); `check_v1.sh` asserts that the topic publishes, not a specific rate.

How to rerun (from the repo root on the host):
```bash
docker compose -f docker/docker-compose.yml run --rm ros /workspace/vision-bot-project/scripts/check_v1.sh
```
