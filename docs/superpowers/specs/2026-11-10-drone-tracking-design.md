# Drone detection and tracking: design

**Date:** 2026-11-10
**Status:** design agreed in chat, written for review
**Builds on:** v2 (two robots on `nav_world`, AMCL localization, dashboard map), the C++ perception node (`vision_bot_perception`)

## 1. Goal

Each robot carries an upward-facing camera that detects a drone flying over the map. The two robots' sightings are fused into one live 3D track of the drone (position, velocity, uncertainty), and the dashboard shows it on the map. The robots keep doing their own jobs; tracking does not steer them.

## 2. Decisions

What Zach chose (2026-11-10):

| Question | Choice |
|---|---|
| What a robot does with the drone | Detect and report: estimate and publish a track, shown on the dashboard |
| Sensor | An upward camera per robot, with OpenCV detection |
| How the drone flies | Random paths, different each run |
| Where the track is shown | The dashboard map, over the rosbridge connection |
| Fusion | One shared 3D Kalman filter, updated by bearings from either robot |

Assumptions made in the design (correct them at review):

- Simulation only, in Gazebo Classic, with the existing two robots on `nav_world`. No hardware.
- One drone at a time.
- The flight is random but seeded. Each run uses a fresh seed and logs it; benchmarks and CI pass a fixed one, so a flight can be replayed exactly. This keeps "different each run" while making accuracy measurable.
- The detector and tracker are C++, following the two C++ ports. Simulation-only helpers, such as the flight node and the benchmark, are Python.
- Everything is off by default and switched on with `drone:=true`, so existing launches, `check_v1.sh` and the v2 benchmarks are unchanged.

## 3. Architecture

```
Gazebo: drone model, moved by drone_flight (seeded random waypoints)
        │ /drone/ground_truth  (scoring only; never read by the tracker)
        ▼
robot1: /robot1/sky_camera/image_raw ──► drone_detector (robot1) ──► /drone/bearings ──┐
robot2: /robot2/sky_camera/image_raw ──► drone_detector (robot2) ──► /drone/bearings ──┤
                                                                                         ▼
                                                    drone_tracker: 3D EKF, one track
                                                                                         ▼
                                     /drone/track ──rosbridge──► dashboard MapView
```

### New packages

| Package | Contents |
|---|---|
| `vision_bot_interfaces` (ament_cmake, rosidl) | `BearingMeasurement.msg`, `DroneTrack.msg` |
| `vision_bot_tracking` (ament_cmake, C++) | `drone_detector` and `drone_tracker`, each a ROS-free library plus an rclcpp component with its own `main`, the layout `vision_bot_perception` uses |
| `vision_bot_drone_sim` (ament_python) | `drone_flight` node, drone model, the world variant with the state plugin, `drone.launch.py` |

### Messages

`BearingMeasurement.msg`:

| Field | Meaning |
|---|---|
| `std_msgs/Header header` | Image stamp; `frame_id` is `map` |
| `string robot` | `robot1` or `robot2` |
| `geometry_msgs/Point origin` | Camera position in the map frame at the image stamp |
| `geometry_msgs/Vector3 direction` | Unit vector from the camera toward the drone, in the map frame |
| `float64 sigma_rad` | One-sigma angular noise of this bearing (pixel noise plus the robot's localization uncertainty) |
| `float64 blob_area_px` | Size of the detected blob, for diagnostics |

`DroneTrack.msg`:

| Field | Meaning |
|---|---|
| `std_msgs/Header header` | `frame_id` is `map` |
| `uint8 status` | `TENTATIVE=1`, `CONFIRMED=2`, `COASTING=3`, `LOST=4` |
| `geometry_msgs/Point position` | Estimated position |
| `geometry_msgs/Vector3 velocity` | Estimated velocity |
| `float64[36] covariance` | 6×6 covariance of (x, y, z, vx, vy, vz), row-major |
| `string[] robots` | Robots whose bearings updated the track in the last second |
| `float64 seconds_since_update` | Time since the last accepted bearing |
| `uint32 rejected` | Bearings rejected by the gate since the track started |

## 4. Components

### 4.1 Drone and flight (`vision_bot_drone_sim`)

- **Model:** a simple quadcopter of boxes and cylinders, about 0.4 m across, dark against the sky, with no physics (`static` with a kinematic pose).
- **World:** `nav_world_drone.world` is a copy of `nav_world.world` that adds Gazebo's `libgazebo_ros_state.so`, so `/set_entity_state` exists. `nav_world.world` itself is unchanged.
- **`drone_flight`:**
  - It draws waypoints uniformly inside the map bounds, at 2–6 m altitude and 0.5–2 m/s per leg, from a `seed` parameter. The default is a fresh seed, logged at startup as `drone_flight seed=<n>`.
  - It moves the drone in straight lines between waypoints, updating the pose at 30 Hz with `/set_entity_state`.
  - It publishes `/drone/ground_truth` as a `geometry_msgs/PoseStamped` and the true velocity on `/drone/ground_truth_velocity`.
  - The path is a pure function of `(seed, bounds, limits, time)`, so it can be tested without Gazebo.

### 4.2 Sky camera

- A second camera link on each robot (`sky_camera_link`), mounted on top of the chassis and pointing straight up.
- Horizontal field of view 2.0 rad (about 115°), 320×240 at 15 Hz.
  - The resolution is kept low because the development machine runs out of memory with two robots, the fleet stack and a browser (DECISIONS #59, #72, #78).
  - At 6 m a 0.4 m drone still spans about 10 pixels.
  - The resolution is a parameter, and the benchmark records the value used.
- The camera is added only when the robot description's `with_sky_camera` argument is true; `drone:=true` sets it.

### 4.3 `drone_detector` (C++, one per robot)

- **Library:** `detect_drone(const cv::Mat & bgr, const DroneDetectParams &) -> std::optional<PixelDetection>`.
  - It thresholds pixels much darker than the sky (sky brightness is estimated from the frame's median).
  - It opens the mask, takes the largest contour within an area range, and returns the centroid and area.
  - It takes no ROS types, the same as `detectors.cpp`.
- **Geometry:**
  - `pixel_to_ray(u, v, K)` gives a unit ray in the camera's optical frame from the intrinsics in `camera_info`.
  - The ray is transformed to the map frame with TF (`sky_camera_optical_frame` → `map` at the image stamp).
  - The camera position from the same transform is the bearing's origin.
- **Noise:** `sigma_rad` = pixel noise (default 1.5 px, converted to radians) combined with an angular term from the robot's AMCL position covariance at the drone's typical range (parameter, default 4 m).
- **Output:** a `BearingMeasurement` on `/drone/bearings` per detection. Nothing is published for a frame without a detection.
- **Failure handling:**
  - When the TF lookup fails (no map pose yet), the frame is skipped, with a throttled warning.
  - When `camera_info` hasn't arrived, the detector waits for it.

### 4.4 `drone_tracker` (C++, one central node)

- **State and model:**
  - State x = (px, py, pz, vx, vy, vz) in the map frame.
  - Constant velocity with white-noise acceleration. `accel_sigma` defaults to 1.0 m/s², a parameter covering the flight's speed changes at waypoints.
- **Measurement:**
  - z = (azimuth, elevation) of the drone seen from the bearing's origin, h(x) = (atan2(dy, dx), atan2(dz, √(dx² + dy²))).
  - Noise is `sigma_rad`² on both components.
  - Azimuth residuals are wrapped to (−π, π].
- **Order:** bearings are processed in stamp order. A bearing older than the filter's time by more than `max_lag_s` (0.2 s) is dropped and counted.
- **Initialization:**
  - It needs two bearings from different robots within `init_window_s` (0.5 s) whose sight lines are at least `min_init_angle_deg` (10°) apart.
  - The starting position is the midpoint of the closest points between the two lines, and only if that point is above the ground and within 20 m.
  - Starting covariance: 4 m² on position and 4 (m/s)² on velocity.
- **Gating:** a bearing whose normalized innovation squared exceeds the 99% chi-square value for 2 degrees of freedom (9.21) is rejected and counted.
- **Status:**
  - *Tentative* after initialization, *confirmed* after `confirm_updates` (5) accepted bearings.
  - *Coasting* when nothing has been accepted for 0.5 s, *lost* after `lost_after_s` (3.0 s).
  - A lost track is published once with status `LOST` and cleared. The next pair of bearings starts a new track.
- **Output:** `/drone/track` at 10 Hz while a track exists, predicted to the publish time.
- **Library:** the filter (predict, update, gate, init) is a ROS-free class tested on its own. The node only converts messages and runs the timer.

### 4.5 Dashboard

- `useDroneTrack` hook: subscribes to `/drone/track` (`vision_bot_interfaces/DroneTrack`) over the existing rosbridge connection (`useRosConnection`).
- `MapView` draws, in map-pixel coordinates via `mapTransform.ts`:
  - a drone marker;
  - the 1-sigma horizontal ellipse from the position covariance's xy block;
  - a 10 s trail;
  - an altitude label;
  - the status, with tentative and coasting drawn dashed.
- Pure helpers `covarianceEllipse(cov2x2)` and `trackToOverlay(track, transform)` are vitest-tested.
- When rosbridge isn't connected or no track exists, nothing is drawn and the rest of the map works as today.

### 4.6 Launch

`drone.launch.py` (in `vision_bot_drone_sim`) starts:
- `drone_flight`;
- a `drone_detector` per robot namespace;
- `drone_tracker`;
- rosbridge, if `bridge:=true`.

`multi_robot.launch.py` gains `drone:=false|true`. When true, it uses `nav_world_drone.world`, adds the sky cameras, spawns the drone and includes `drone.launch.py`.

## 5. Testing

**Unit (gtest):**
- `pixel_to_ray` on known intrinsics (center pixel straight up; corner pixels at the field-of-view edge).
- The closest point between two lines, including parallel and nearly parallel lines (rejected).
- `detect_drone` on rendered sky frames: drone found within 2 px of the true centroid; empty sky gives no detection; a blob outside the area range is ignored.
- The filter, on synthetic bearings from two fixed positions watching a known path, with seeded noise:
  - **Convergence:** position error under 3σ after 20 updates.
  - **Consistency:** the average normalized estimation error squared over 50 seeded runs falls inside the 95% chi-square band.
  - **Coasting and loss:** status changes at the configured times.
  - **Rejection:** injected outlier bearings are rejected and counted.
  - **One-robot behavior:** with bearings from one robot only, the uncertainty grows along the line of sight.
- `drone_flight`'s path function (Python): the same seed gives the same path; positions stay within bounds and limits.

**Launch tests:**
- The detector, given a sky image plus a static TF and `camera_info`, publishes the expected bearing.
- The tracker, fed bearings, publishes the status sequence tentative → confirmed → coasting → lost.
- A bad configuration (for example a negative `accel_sigma`) exits with status 1.

**Simulation benchmark:** `benchmarks/drone_tracking.py`.
- **Setup:** `multi_robot.launch.py drone:=true`, N seeded flights (default 5 seeds × 120 s), robots parked at their spawn poses.
- **Scoring:** against `/drone/ground_truth`, interpolated to the track's stamps.
- **Reported:**
  - 3D position error (mean, p95);
  - altitude error;
  - the share of time a confirmed track exists while the drone is in at least one camera's view;
  - time to the first confirmed track;
  - rejected bearings.
- **Second run:** the same seeds with robot2's detector off, to measure what the second viewpoint adds.
- **Output:** raw CSV and a markdown summary in `benchmarks/results/`, as with the other benchmarks.

**CI regression:**
- A bag with one seeded flight's `/drone/bearings` and `/drone/ground_truth` (small, no images) is replayed through `drone_tracker` in CI.
- The job fails if the mean 3D error exceeds the recorded baseline plus a margin. The threshold is set from measured replays, as with `benchmarks/baseline.json`.

## 6. Success criteria

- All unit, launch and dashboard tests pass, and CI is green with the replay regression in place.
- The benchmark is committed with raw output; README numbers come only from it. No accuracy target is set in advance.
- During a seeded flight the dashboard shows the live track (marker, ellipse, trail, altitude, status).
- `check_v1.sh` passes. With `drone:=false` (the default) the v1 and v2 launches start the same nodes as before.

## 7. Out of scope

- Steering a robot or a camera toward the drone.
- More than one drone, or several tracks at once.
- Fleet API storage of tracks.
- Hardware.
- Learned detectors. The sky is plain in simulation, so contour detection is enough; a DNN can come later behind the same `detect_drone` interface.

## 8. Risks

- **Machine memory:** two more cameras and a moving model add load to a machine that already runs short. Mitigated by the 320×240 resolution, `drone:=false` by default, and benchmark runs with the fleet stack and browser closed.
- **AMCL error leaking into bearings:** a robot's map pose error tilts its bearings. The detector inflates `sigma_rad` from the AMCL covariance, and the benchmark reports altitude error separately, since that is where geometry errors show most.
- **`/set_entity_state` timing:** Gazebo may apply poses late under load, so ground truth is taken from the published commanded pose. The benchmark notes this.
