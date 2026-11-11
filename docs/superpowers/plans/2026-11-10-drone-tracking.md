# Drone detection and tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Each robot's upward camera detects a drone flying seeded random paths, and one 3D EKF fuses both robots' bearings into a live track drawn on the dashboard map.

**Architecture:** Three new packages:
- `vision_bot_interfaces`: the two messages;
- `vision_bot_drone_sim`: the drone model, flight node, world variant and launch file;
- `vision_bot_tracking`: the C++ detector and tracker, each a ROS-free library plus an rclcpp component with its own `main`.

The robot description gains an optional sky camera, `multi_robot.launch.py` gains `drone:=true`, and the dashboard draws `/drone/track` from rosbridge.

**Tech Stack:** ROS 2 Humble, Gazebo Classic 11 (`gazebo_ros_state`), rclcpp, rclcpp_components, tf2_ros, cv_bridge, OpenCV 4, Eigen3, gtest, launch_testing, rosbag2; Next.js, roslib, vitest.

**Spec:** `docs/superpowers/specs/2026-11-10-drone-tracking-design.md`. Section numbers below (§) refer to it.

## Global Constraints

- **Off by default:** `drone:=false` is the default everywhere. With it false, every existing launch starts the same nodes as before, and `check_v1.sh` passes at each gate.
- **Simulation only:** Gazebo Classic, the two robots on `nav_world`, one drone.
- **Message fields:** exactly as in §3 (`BearingMeasurement`, `DroneTrack`, with status constants `TENTATIVE=1`, `CONFIRMED=2`, `COASTING=3`, `LOST=4`).
- **Defaults pinned by the spec:**

  | Area | Values |
  |---|---|
  | Flight | waypoints inside the map bounds, altitude 2–6 m, 0.5–2 m/s per leg, pose updates at 30 Hz |
  | Sky camera | 2.0 rad horizontal FOV, 320×240, 15 Hz |
  | Detector | pixel noise 1.5 px, typical range 4 m |
  | Tracker | `accel_sigma` 1.0 m/s², `max_lag_s` 0.2, `init_window_s` 0.5, `min_init_angle_deg` 10, init point above ground and within 20 m, starting covariance 4 m² and 4 (m/s)², gate 9.21, `confirm_updates` 5, coasting after 0.5 s, `lost_after_s` 3.0, publish at 10 Hz |
- **C++ code:** C++17, `-Wall -Wextra -Wpedantic` with no warnings. Each node has its own `main` that exits 1 when the node can't start, as in `vision_bot_perception/src/perception_main.cpp`.
- **Environment:** everything builds and runs in the Docker image (`docker/docker-compose.yml`, service `ros`).
- **Git:**
  - One branch per task, `v2/drone-N-name`, each branched off the previous one; Task 1 branches off `v2/drone-0-spec`.
  - Push after every commit. No Co-Authored-By trailer.
- **Numbers:** never fabricate any. Benchmark output is committed raw, and README numbers come only from committed results.
- **Gates:** stop and report after Task 5 and after Task 7, appending to `docs/INTERVIEW_NOTES.md` each time.
- **Machine load:** close the browser and the fleet stack during simulation runs. Count `CRITICAL FAILURE|Resetting managed nodes` in launch logs and discard runs where it is nonzero (`docs/BENCHMARKS.md`).

## Review Focus

1. **Drone straight overhead.** At elevation near 90°, azimuth is undefined, and the azimuth row of the measurement Jacobian blows up as 1/horizontal distance. The filter must stay finite. When elevation > 85°, skip the azimuth component and update with elevation alone. Pinned by `TEST(DroneFilter, OverheadBearingStaysFinite)` (Task 4).
2. **Azimuth wrap.** A drone near ±π azimuth from a robot must not produce a 2π residual. Pinned by `TEST(DroneFilter, AzimuthResidualWrapsAcrossPi)` (Task 4).
3. **Only one robot ever sees the drone.** One robot's bearings can't start a track (§4.4 needs two robots), so no track appears and nothing crashes. The dashboard shows nothing. Pinned by `TEST(DroneFilter, SingleRobotNeverInitializes)` (Task 4).
4. **Sim time.** All nodes run with `use_sim_time`, and bearings carry sim stamps. The tracker's lag check and publish timer must use the node clock, not wall time, or every bearing looks stale. Pinned by the Task 4 launch test publishing `/clock` and running the tracker with `use_sim_time:=true`.
5. **TF not ready at the image stamp** (startup, or AMCL not yet localized). The detector skips the frame with a throttled warning and never publishes a bearing with a stale or identity pose. Pinned by `test_no_bearing_without_tf` in the Task 3 launch test.

---

## File Structure

```
ros2_ws/src/vision_bot_interfaces/        msg/BearingMeasurement.msg, msg/DroneTrack.msg
ros2_ws/src/vision_bot_drone_sim/
  vision_bot_drone_sim/flight_path.py     seeded waypoints + pose at time t (pure)
  vision_bot_drone_sim/drone_flight.py    node: /set_entity_state at 30 Hz, ground truth topics
  models/drone/model.sdf, worlds/nav_world_drone.world, launch/drone.launch.py
  test/test_flight_path.py
ros2_ws/src/vision_bot_tracking/
  include/vision_bot_tracking/{detect.hpp, geometry.hpp, drone_filter.hpp, nodes.hpp}
  src/{detect.cpp, geometry.cpp, drone_filter.cpp, detector_node.cpp, tracker_node.cpp,
       detector_main.cpp, tracker_main.cpp}
  test/{test_detect.cpp, test_geometry.cpp, test_drone_filter.cpp,
        test_detector_node.py, test_tracker_node.py}
dashboard/app/lib/droneTrack.ts (+ .test.ts), dashboard/app/hooks/useDroneTrack.ts
benchmarks/drone_tracking.py, benchmarks/drone_replay.py, benchmarks/bags/drone_flight/
```
Modified:
- `ros2_ws/src/vision_bot_nav/urdf/vision_bot_nav.urdf.xacro` (`with_sky_camera`)
- `nav_sim.launch.py`, `multi_robot.launch.py` (`drone`)
- `dashboard/app/components/MapView.tsx`
- `.github/workflows/v2-ci.yml`
- README, `docs/BENCHMARKS.md`, `docs/DECISIONS.md`, `docs/INTERVIEW_NOTES.md`

---

### Task 1: Messages and the flying drone

Branch `v2/drone-1-sim` off `v2/drone-0-spec`. Commit this plan first.

**Files:** `vision_bot_interfaces/` (package.xml, CMakeLists.txt, two `.msg` files), `vision_bot_drone_sim/` (package.xml, setup.py, `flight_path.py`, `drone_flight.py`, model, world, `test/test_flight_path.py`).

**Interfaces:**
- **Produces:**
  - messages `vision_bot_interfaces/msg/BearingMeasurement` and `DroneTrack` (fields as in §3).
  - `flight_path.py`:
    - `make_waypoints(seed: int, bounds: tuple[float,float,float,float], alt=(2.0, 6.0), speed=(0.5, 2.0), n: int = 50) -> list[Waypoint]`, where `Waypoint` is a `(x, y, z, speed)` NamedTuple;
    - `pose_at(waypoints, t: float) -> tuple[tuple[float,float,float], tuple[float,float,float]]`, giving position and velocity at time t, moving in straight lines leg by leg and wrapping to the start after the last leg.
  - `drone_flight` node:
    - publishes `/drone/ground_truth` (`PoseStamped`, frame `map`) and `/drone/ground_truth_velocity` (`Vector3Stamped`);
    - logs `drone_flight seed=<n>`;
    - parameters: `seed` (default -1 = draw from `os.urandom`), `bounds` (default: the `nav_world` map extent read from `maps/nav_world.yaml` and its image size), `rate_hz` 30.
  - World `nav_world_drone.world`: `nav_world.world` plus `<plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">`. Model `drone`: static, about 0.4 m across, dark gray.

- [ ] **Step 1: Failing tests** in `test/test_flight_path.py`:
  - `test_same_seed_same_path`: two calls with seed 7 are equal.
  - `test_different_seed_differs`.
  - `test_waypoints_within_limits`: every waypoint is inside the bounds, with z in [2, 6] and speed in [0.5, 2].
  - `test_pose_moves_at_leg_speed`: the distance covered over 0.1 s matches the leg's speed to 1e-9 away from leg ends.
  - `test_pose_wraps_after_last_leg`.

  Run `python3 -m pytest ros2_ws/src/vision_bot_drone_sim/test` and see them fail (module missing).
- [ ] **Step 2: Implement `flight_path.py`** with `random.Random(seed)`. Run the tests and see them pass.
- [ ] **Step 3: Messages and model.** Add the `vision_bot_interfaces` package (rosidl, depends on `std_msgs` and `geometry_msgs`), the drone model and the world variant. Verify:
  - `colcon build --packages-select vision_bot_interfaces vision_bot_drone_sim` succeeds;
  - `ros2 interface show vision_bot_interfaces/msg/DroneTrack` lists the §3 fields and constants.
- [ ] **Step 4: `drone_flight` node.**
  - It calls `/set_entity_state` (`gazebo_msgs/srv/SetEntityState`) asynchronously at `rate_hz` with the pose from `pose_at(waypoints, sim_time - t0)`.
  - It publishes ground truth with the same stamp and pose.
  - It spawns the model through `/spawn_entity` if `drone` isn't present.
- [ ] **Step 5: Live check in Docker.**
  - Run `gzserver` with `nav_world_drone.world` plus `ros2 run vision_bot_drone_sim drone_flight --ros-args -p seed:=42 -p use_sim_time:=true`.
  - Within 30 s of startup, `gz model -m drone -p` must change across three polls taken 2 s apart, and `/drone/ground_truth` must publish at about 30 Hz.
  - Rerunning with seed 42 must log the same first waypoint.
- [ ] **Step 6: Commit and push.**

### Task 2: Sky cameras and `drone:=true` wiring

Branch `v2/drone-2-cameras` off Task 1.

**Files:**
- Modify: `vision_bot_nav/urdf/vision_bot_nav.urdf.xacro`, `nav_sim.launch.py`, `multi_robot.launch.py`
- Create: `vision_bot_drone_sim/launch/drone.launch.py` (flight only for now)
- Test: `vision_bot_nav/test/test_robot_description.py`

**Interfaces:**
- **Consumes:** the Task 1 world and node.
- **Produces:**
  - Sky camera:
    - the xacro argument `with_sky_camera` (default `false`);
    - links `sky_camera_link` and `sky_camera_optical_frame` (standard optical frame: z forward = up, x right, y down);
    - topics `<ns>/sky_camera/image_raw` and `<ns>/sky_camera/camera_info`.
  - Launch arguments `drone` on `nav_sim.launch.py` (passes `with_sky_camera`) and on `multi_robot.launch.py`. With `drone:=true`, `multi_robot.launch.py` uses `nav_world_drone.world`, turns the sky cameras on, and includes `drone.launch.py`.
  - Launch argument `seed` (default `-1`, meaning random) on `multi_robot.launch.py` and `drone.launch.py`, passed to `drone_flight`.

- [ ] **Step 1: Failing tests** in `test_robot_description.py`, which run `xacro` on the file:
  - `test_no_sky_camera_by_default`: no link named `sky_camera_link`.
  - `test_sky_camera_points_up`: with `with_sky_camera:=true`, the camera joint's rpy rotates the sensor's x axis to world +z (pitch −π/2). The sensor block has `horizontal_fov` 2.0, width 320, height 240, update_rate 15.
  - `test_existing_links_unchanged`: the set of links with `with_sky_camera:=false` equals the set from `git show HEAD:` of the file.

  Run them and see them fail.
- [ ] **Step 2: Implement the xacro change.** Run the tests and see them pass.
- [ ] **Step 3: Launch arguments.** Extend `test_impl_launch.py`-style loading:
  - `test_drone_off_by_default`: `multi_robot.launch.py` declares `drone` with default `false`, and with the default no node from `vision_bot_drone_sim` is in the description.
  - `test_drone_true_uses_drone_world`.

  Run them and see them fail, implement, and see them pass.
- [ ] **Step 4: Live check.**
  - With `ros2 launch vision_bot_nav multi_robot.launch.py gui:=false drone:=true`, both `/robot1/sky_camera/image_raw` and `/robot2/sky_camera/image_raw` publish at about 15 Hz.
  - Save one frame from each as PNG to `ros2_ws/src/vision_bot_tracking/test/fixtures/` while the drone is in view (from `ros2 topic echo --once` through cv_bridge). These are the real-render fixtures for Task 3.
  - With `drone:=false`, `ros2 node list` matches the list from `main`.
- [ ] **Step 5: `check_v1.sh` passes. Commit and push.**

### Task 3: `drone_detector`

Branch `v2/drone-3-detector` off Task 2.

**Interfaces:**
- **Consumes:** `BearingMeasurement`, the sky camera topics and frames (Task 2), and fixtures from Task 2.
- **Produces** (namespace `vision_bot_tracking`):
  ```cpp
  struct DroneDetectParams { double darkness = 40.0; double min_area_px = 6.0; double max_area_px = 4000.0; };
  struct PixelDetection { double u, v, area_px; };
  std::optional<PixelDetection> detect_drone(const cv::Mat & bgr, const DroneDetectParams & p);
  Eigen::Vector3d pixel_to_ray(double u, double v, const Eigen::Matrix3d & K);  // unit, optical frame
  double bearing_sigma(double pixel_sigma_px, double fx, double pos_var_m2, double typical_range_m);
  ```
  - `detect_drone`: grayscale; the sky level is the median; the mask holds pixels with sky − value > `darkness`; morphological open with 3×3; largest contour within the area range; area-weighted centroid.
  - `bearing_sigma`: `sqrt((pixel_sigma_px / fx)^2 + pos_var_m2 / typical_range_m^2)`, where `pos_var_m2` is the larger of AMCL's x and y variances.
  - Node `drone_detector`:
    - parameters `robot` (string, required), `pixel_sigma_px` 1.5, `typical_range_m` 4.0, and the `DroneDetectParams` fields;
    - subscribes `sky_camera/image_raw`, `sky_camera/camera_info` and `amcl_pose`, relative to its namespace;
    - publishes `/drone/bearings` (absolute topic).

- [ ] **Step 1: Failing gtests:**
  - `test_detect.cpp`:
    - `TEST(DetectDrone, FindsDarkBlobOnSky)`: a synthetic 320×240 sky (BGR 200, 170, 140) with a filled dark circle radius 4 at (100, 60) gives u, v within 1 px.
    - `TEST(DetectDrone, EmptySkyGivesNothing)`.
    - `TEST(DetectDrone, IgnoresBlobsOutsideAreaRange)`.
    - `TEST(DetectDrone, FindsDroneInRenderedFrames)`: the Task 2 PNGs give a detection, with expected centroids recorded in `fixtures/rendered.json` when the PNGs were saved, within 2 px.
  - `test_geometry.cpp`:
    - `TEST(PixelToRay, CenterPixelIsOpticalAxis)` (ray = (0, 0, 1)).
    - `TEST(PixelToRay, EdgePixelIsAtHalfFov)` (for K from a 2.0 rad FOV at 320 px, u = 0 gives a ray 1.0 rad from the axis, to 1e-9).
    - `TEST(BearingSigma, CombinesPixelAndPoseTerms)`.

  Build and see them fail.
- [ ] **Step 2: Implement `detect.cpp` and `geometry.cpp`.** See the tests pass with no warnings.
- [ ] **Step 3: Failing launch test** `test_detector_node.py`. The detector is started with `robot:=robot1` under namespace `robot1`; the test publishes `camera_info` for the 320×240 / 2.0 rad camera, a static TF `map → sky_camera_optical_frame` at (1, 2, 0.2) pointing up, and the synthetic sky frame.
  - `test_publishes_bearing`: origin = (1, 2, 0.2) to 1e-6; direction within 0.01 rad of the ray computed in the test; `robot == "robot1"`; `sigma_rad > 0`.
  - `test_no_bearing_for_empty_sky`.
  - `test_no_bearing_without_tf` (Review Focus 5): with the TF not published, there is no bearing in 2 s and the process stays up.
  - `test_missing_robot_param_exits_1`.

  Run it and see it fail.
- [ ] **Step 4: Implement `detector_node.cpp` and `detector_main.cpp`.**
  - TF lookup of `map ← <camera frame_id>` at the image stamp with a 0.05 s timeout.
  - `pos_var_m2` from the last `amcl_pose`; 0 if none has arrived.
  - See all tests pass.
- [ ] **Step 5: Commit and push.**

### Task 4: `drone_tracker`

Branch `v2/drone-4-tracker` off Task 3.

**Interfaces:**
- **Consumes:** `BearingMeasurement`, `DroneTrack`.
- **Produces:**
  ```cpp
  struct Bearing { double t; std::string robot; Eigen::Vector3d origin, direction; double sigma; };
  struct FilterParams { double accel_sigma = 1.0, max_lag_s = 0.2, init_window_s = 0.5,
                        min_init_angle_deg = 10.0, max_init_range_m = 20.0, gate = 9.21,
                        lost_after_s = 3.0, coast_after_s = 0.5; int confirm_updates = 5; };
  enum class Status { None = 0, Tentative = 1, Confirmed = 2, Coasting = 3, Lost = 4 };
  class DroneFilter {
  public:
    explicit DroneFilter(FilterParams p);     // throws std::invalid_argument on a non-positive accel_sigma, gate or lost_after_s
    enum class Result { Applied, Initialized, Buffered, Rejected, Stale };
    Result add(const Bearing & b);             // bearings in any order; stale ones (older than now - max_lag_s) are refused
    Status status(double t) const;             // Coasting/Lost from the time since the last applied bearing
    Eigen::Matrix<double,6,1> state_at(double t) const;     // predicted to t
    Eigen::Matrix<double,6,6> covariance_at(double t) const;
    std::vector<std::string> robots_since(double t) const;
    uint32_t rejected() const;
    void reset();                              // called after Lost is published
  };
  Eigen::Vector3d closest_point(const Eigen::Vector3d & o1, const Eigen::Vector3d & d1,
                                const Eigen::Vector3d & o2, const Eigen::Vector3d & d2);  // midpoint of the closest points
  ```
  - **Measurement model:** as §4.4. When elevation > 85°, update with the elevation only (Review Focus 1).
  - **Update form:** Joseph form, so the covariance stays symmetric positive definite.
  - **Node `drone_tracker`:** subscribes `/drone/bearings` and publishes `/drone/track` at 10 Hz from a timer on the node clock (Review Focus 4). It publishes once with `LOST` and then calls `reset()`.

- [ ] **Step 1: Failing gtests** in `test_drone_filter.cpp`. The helper `simulate(path, robots, seed, noise_rad)` generates bearings from robots at (0, 0, 0.2) and (−2.5, 0, 0.2) watching a path at 15 Hz each.
  - `ClosestPointOfCrossingLines`
  - `ParallelLinesDoNotInitialize` (angle < 10°)
  - `InitializesFromTwoRobots`
  - `SingleRobotNeverInitializes` (Review Focus 3)
  - `ConvergesOnStraightPath` (error < 3σ after 20 updates)
  - `ConsistentOverSeededRuns`: mean NEES over 50 seeds × 200 steps is inside the 95% chi-square band for 6 degrees of freedom × 50 runs.
  - `ConfirmsAfterFiveUpdates`
  - `CoastsThenLost` (0.5 s and 3.0 s)
  - `RejectsOutlier` (a bearing 20° off is rejected and `rejected() == 1`)
  - `StaleBearingRefused`
  - `OutOfOrderWithinLagApplied`
  - `AzimuthResidualWrapsAcrossPi` (Review Focus 2)
  - `OverheadBearingStaysFinite` (Review Focus 1)
  - `OneRobotGrowsUncertaintyAlongLineOfSight`: after robot2 stops, the covariance's largest eigenvector is within 15° of robot1's line of sight.
  - `InvalidParamsThrow`

  See them fail.
- [ ] **Step 2: Implement `drone_filter.cpp`.** See all pass with no warnings.
- [ ] **Step 3: Failing launch test** `test_tracker_node.py`. The tracker runs with `use_sim_time:=true`; the test publishes `/clock` and bearings from two virtual robots watching a point moving at 1 m/s.
  - `test_status_sequence`: TENTATIVE, then CONFIRMED, then (bearings stop) COASTING, then LOST once, then no more tracks.
  - `test_track_near_truth`: confirmed position within 0.5 m of the true point.
  - `test_bad_param_exits_1` (`accel_sigma:=-1`).

  See it fail.
- [ ] **Step 4: Implement `tracker_node.cpp` and `tracker_main.cpp`.** See all tests pass.
- [ ] **Step 5: Commit and push.**

### Task 5: End to end in simulation, then Gate A

Branch `v2/drone-5-e2e` off Task 4.

- [ ] **Step 1: Complete `drone.launch.py`.** It starts:
  - `drone_flight` (passing `seed`);
  - `drone_detector` in `robot1` and `robot2`;
  - `drone_tracker`.

  All with `use_sim_time:=true`. Add a `bridge` argument (default `false`) that includes `vision_bot/launch/bridge_launch.py`. Extend the Task 2 launch tests: `test_drone_true_starts_detectors_and_tracker` (two `drone_detector` nodes in the robot namespaces, one `drone_tracker`).
- [ ] **Step 2: Live run.** `multi_robot.launch.py gui:=false drone:=true seed:=42` for 180 s, recording `/drone/track` and `/drone/ground_truth` with `ros2 bag record`. Then:
  - a confirmed track appears;
  - its position is within 1 m of ground truth for most confirmed samples (a sanity check only; Task 7 measures accuracy);
  - the launch log has 0 `CRITICAL FAILURE`.
- [ ] **Step 3: Regression checks.**
  - `check_v1.sh` passes.
  - `multi_robot.launch.py` with `drone:=false` starts the same node list as `main`.
- [ ] **Step 4: Record DECISIONS #89 onward and commit.**
  - The overhead elevation-only update.
  - Ground truth is the commanded pose.
  - Tracks only start from two robots.
- [ ] **Gate A:** report the tests, the live run, `check_v1.sh` and CI. Append to `docs/INTERVIEW_NOTES.md`. Stop.

### Task 6: Dashboard

Branch `v2/drone-6-dashboard` off Task 5.

**Interfaces:**
- **Consumes:** the `DroneTrack` fields; `worldToPixel` and `MapMeta` from `app/lib/mapTransform.ts`.
- **Produces:**
  - `droneTrack.ts`:
    - `covarianceEllipse(cxx, cxy, cyy): { rx: number; ry: number; angleDeg: number }` (1-sigma);
    - `trackToOverlay(track: DroneTrackMsg, meta: MapMeta): { cx, cy, rxPx, ryPx, angleDeg, altitudeM, dashed: boolean } | null` (null for LOST).
  - `useDroneTrack(ros)`: the latest message plus a 10 s trail of map-pixel points, cleared on LOST.

- [ ] **Step 1: Failing vitest cases** in `droneTrack.test.ts`:
  - a diagonal covariance gives axis-aligned radii √c;
  - a covariance with a correlation term gives the eigen-decomposition angle;
  - a zero-area covariance doesn't produce NaN;
  - tentative and coasting give `dashed: true`;
  - LOST gives null;
  - pixel conversion matches `worldToPixel`.

  See them fail.
- [ ] **Step 2: Implement `droneTrack.ts`** and see the tests pass.
- [ ] **Step 3: Draw it.** Add the hook, and draw in `MapView` (marker, ellipse, trail polyline, `"{z} m"` label, status text). Draw nothing when rosbridge is disconnected.
- [ ] **Step 4: Verify in the browser pane.**
  - Live sim with `drone:=true bridge:=true`, dashboard on its production build.
  - Screenshot the track on the map, and check `read_console_messages` has no errors.
  - With the sim stopped, the map still loads robots and goals.
- [ ] **Step 5: `npm test` and `npm run build` pass. Commit and push.**

### Task 7: Benchmark, CI regression, docs, then Gate B

Branch `v2/drone-7-benchmark` off Task 6.

**Interfaces:**
- **Produces:**
  - `benchmarks/drone_tracking.py` with pure helpers:
    - `score(track_rows, truth_rows) -> dict` with keys `n`, `mean_err_m`, `p95_err_m`, `mean_alt_err_m`, `tracked_fraction`, `time_to_first_confirmed_s`, `rejected`;
    - `in_view(truth_xyz, robot_xyz, fov_rad) -> bool`.
  - `benchmarks/drone_replay.py`: replays `benchmarks/bags/drone_flight` (bearings plus ground truth, zstd) through `drone_tracker` and exits 1 above `drone.threshold_m` in `benchmarks/baseline.json`.

- [ ] **Step 1: Failing pytest cases** in `benchmarks/tests/test_drone_tracking.py`:
  - `score` on hand-made rows (known errors give known mean and p95; truth is interpolated to track stamps);
  - `tracked_fraction` counts only in-view time;
  - `in_view` at the field-of-view edge.

  Implement them and see them pass.
- [ ] **Step 2: Benchmark runs.** Seeds 1–5, 120 s each, robots parked; then the same seeds with `robot2`'s detector off (`drone_tracking.py --robots robot1`). Commit raw CSV and markdown results.
- [ ] **Step 3: Regression baseline.**
  - Record `bags/drone_flight` with seed 42 (only `/drone/bearings`, `/drone/ground_truth`, `/clock`).
  - Replay it 5 times; the threshold is the mean plus 0.05 m, written to `baseline.json` with the raw replay output committed.
  - Add the replay step to `v2-ci.yml`, plus the new packages in the build and test lines.
  - CI goes green.
- [ ] **Step 4: Docs.**
  - **README:** a drone-tracking section with a results table taken from the committed files, and how to run it.
  - **`docs/BENCHMARKS.md`:** section 7.
  - **DECISIONS** and **INTERVIEW_NOTES:** Gate B notes.
  - Run the clean-user-facing-text check on the new prose.
- [ ] **Step 5: Commit and push; CI green.**
- [ ] **Gate B:** report the numbers from the committed files, including one robot vs two. Ask about a PR. Stop.
