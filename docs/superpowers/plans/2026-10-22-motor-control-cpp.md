# motor_control_node in C++ Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A C++ (rclcpp) `motor_control_node` that is a drop-in replacement for the Python one, with the same node name, topics, parameters and control law, giving the same commands for the same detections. It is selectable at launch with `control_impl`, so the whole v1 pipeline can run in C++.

**Architecture:** A new ament_cmake package `vision_bot_control`, following `vision_bot_perception`. A ROS-free library holds `compute_tracking_cmd`, and gtest checks it against reference outputs from the Python function. An rclcpp component (`vision_bot_control::MotorControlNode`) wraps it: detection callback, enable switch, 10 Hz lost-target watchdog. It is also built as a `motor_control_node` executable with its own `main`. A launch argument `control_impl` (`python` | `cpp`) picks the package.

**Tech Stack:** ROS 2 Humble, rclcpp, rclcpp_components, std_msgs, geometry_msgs, gtest, nlohmann_json, launch_testing, Python 3 for fixtures.

**Spec:** the Python node itself (`ros2_ws/src/vision_bot/vision_bot/motor_control_node.py`, its docstring and `test/test_motor_control_logic.py`), plus the user's request on 2026-10-22: "port motor_control_node to C++ next". This plan follows the pattern of `docs/superpowers/plans/2026-10-18-perception-cpp.md`.

## Global Constraints

- The Python `motor_control_node` is not changed. v1 is never broken: `scripts/check_v1.sh` passes at the gate on every combination it is run with.
- **Same interface as the Python node:**
  - Node name: `motor_control_node`.
  - Subscribes: `/vision_bot/detection` (`std_msgs/Float32MultiArray`, `[detected, offset_x, offset_y, area_fraction, confidence]`) and `/vision_bot/autonomous_enabled` (`std_msgs/Bool`).
  - Publishes: `/cmd_vel` (`geometry_msgs/Twist`).
  - QoS depth 10 on all three.
- **Same parameters, with these defaults** (all doubles except `search_when_lost`, a bool), read on every callback as the Python node does:

  | Parameter | Default |
  |---|---|
  | `base_linear_speed` | 0.15 |
  | `angular_gain` | 1.0 |
  | `offset_deadband` | 0.05 |
  | `lost_timeout_sec` | 0.75 |
  | `search_when_lost` | true |
  | `search_angular_speed` | 0.3 |
  | `stop_area_fraction` | 0.15 |
- **Same behavior as the Python node:**
  - **Detections:** a detection with `detected < 0.5` publishes nothing and does not refresh the last-detection time.
  - **Last seen direction:** updates only when `|offset_x| >= offset_deadband`.
  - **Disabled:** while autonomous control is disabled, nothing is published (neither callback nor watchdog).
  - **Watchdog:** every 0.1 s, if more than `lost_timeout_sec` has passed since the last real detection, it publishes zero linear speed and, when `search_when_lost` is true, `angular.z = search_angular_speed * last_seen_direction` (initial direction +1). At startup it treats the last detection as long ago, as Python does with its 0.0 start.
- C++17, `-Wall -Wextra -Wpedantic` with no warnings.
- Everything builds and runs in the Docker image. One branch per task (`v2/cpp-N-name`), each branched off the previous one, starting from `v2/cpp-6-default` (PR #4 is not merged yet). Push after every commit. No Co-Authored-By trailer.
- Gate after Task 2 with a report, appended to `docs/INTERVIEW_NOTES.md`.

## Review Focus

1. **Malformed detection messages (not 5 floats).** Python's tuple unpack raises, which crashes the node out of `spin`. The C++ node should not crash: it ignores the message and logs a throttled warning, and that is a deliberate, recorded difference. Pinned by a launch-test case.
2. **The watchdog clock.** Python uses wall time (`time.time()`); C++ uses `std::chrono::steady_clock`. Elapsed time is the same, but steady_clock does not jump when the system clock changes, and it ignores `use_sim_time` just as Python's `time.time()` does. Pinned by the watchdog launch-test case timing a lost target.
3. **Re-enabling after a disable.** Once `/vision_bot/autonomous_enabled` turns true again, the next watchdog tick may publish a search spin straight away, because no detection arrived while disabled. That matches Python. Pinned by a launch-test case.
4. **`stop_area_fraction <= 0`.** Python divides by it only when `area_fraction < stop_area_fraction`, so for 0 it never divides (area is >= 0, linear speed is 0). C++ must take the same branch, never compute 0/0. Covered by a fixture case with `stop_area_fraction = 0.0`.
5. **The search spin direction.** Python sets `last_seen_direction = +1` for a target last seen on the right, and searches with `angular.z = +search_angular_speed * direction`. Positive z turns left, while the control law turns right toward a right-side target (`angular.z = -gain * offset_x`). So the search turns away from where the target was last seen. The port matches Python, and the gate asks whether to fix both nodes.
6. **`offset_x` beyond ±1** (a future detector, or a malformed but 5-long message). Python caps the slowdown at `min(1, |offset_x|)`, so linear speed stays >= 0, and angular speed is not clamped. Covered by fixture cases at offset ±1.5.

---

## File Structure

```
ros2_ws/src/vision_bot_control/
  package.xml, CMakeLists.txt
  include/vision_bot_control/tracking.hpp       TrackingParams, TrackingCmd, compute_tracking_cmd (no ROS)
  include/vision_bot_control/motor_control_node.hpp   make_motor_control_node
  src/tracking.cpp
  src/motor_control_node.cpp                     rclcpp component MotorControlNode
  src/motor_control_main.cpp                     executable main: exits 1 if the node cannot start
  test/test_tracking.cpp                         gtest against the Python reference fixture
  test/fixtures/reference.json
  test/test_motor_control_node.py                launch_testing: topics, enable switch, watchdog, malformed input
scripts/make_control_fixtures.py (+ scripts/tests/test_make_control_fixtures.py)
```
Modified: `sim_launch.py`, `nav_sim.launch.py`, `hardware_launch.py` (one launch argument each), `ros2_ws/src/vision_bot_nav/test/test_perception_impl_launch.py` (extended to `control_impl`), `.github/workflows/v2-ci.yml`, README, `docs/DECISIONS.md`, `docs/INTERVIEW_NOTES.md`.

---

### Task 1: Package skeleton, reference fixture and the control law

Branch `v2/cpp-7-control-law` off `v2/cpp-6-default`. Commit this plan first.

**Files:** `ros2_ws/src/vision_bot_control/{package.xml,CMakeLists.txt}`, `include/vision_bot_control/tracking.hpp`, `src/tracking.cpp`, `test/test_tracking.cpp`, `test/fixtures/reference.json`, `scripts/make_control_fixtures.py`, `scripts/tests/test_make_control_fixtures.py`.

**Interfaces:**
- Produces (namespace `vision_bot_control`):
  ```cpp
  struct TrackingParams { double angular_gain = 1.0, base_linear_speed = 0.15,
                          offset_deadband = 0.05, stop_area_fraction = 0.15; };
  struct TrackingCmd { double linear_x = 0.0, angular_z = 0.0; };
  TrackingCmd compute_tracking_cmd(double offset_x, double area_fraction, const TrackingParams & p);
  ```
- Fixture format: `{"cases": [{"offset_x", "area_fraction", "params": {angular_gain, base_linear_speed, offset_deadband, stop_area_fraction}, "expected": [linear_x, angular_z]}]}`.

- [ ] **Step 1: Generator test** `scripts/tests/test_make_control_fixtures.py::test_writes_reference_grid`. It runs `main(tmp_path)` and asserts:
  - at least 200 cases;
  - every case has both `expected` values;
  - the case `(0.0, 0.0)` at default params gives `[0.15, -0.0]` (the Python result includes the negative zero).

  Run it and see it fail (module missing).
- [ ] **Step 2: `scripts/make_control_fixtures.py`**, with `make_cases() -> list[tuple[float, float, dict]]` and `main(out_dir)`.
  - **Grid:**
    - offset_x in {-1.5, -1.0, -0.5, -0.06, -0.05, -0.049, 0.0, 0.02, 0.05, 0.3, 0.8, 1.0, 1.5};
    - area_fraction in {0.0, 0.01, 0.1, 0.13, 0.1499, 0.15, 0.5, 0.9, 1.0};
    - two param sets: the defaults, and the line-follow tuning (`angular_gain` 1.8, `base_linear_speed` 0.10, `stop_area_fraction` 1.0).
  - **Extra cases:** a `stop_area_fraction` 0.0 set over the area values (Review Focus 4), and `offset_deadband` 0.0.
  - **Output:** expected values come from the Python `compute_tracking_cmd`, written to `reference.json`, which is committed. Run the test and see it pass.
- [ ] **Step 3: Failing gtest** `TEST(TrackingReference, MatchesPythonOnEveryCase)`:
  - linear and angular values within 1e-12 of Python;
  - `std::signbit` equal on `angular_z`, because negative zero must match: it reaches `/cmd_vel` as -0.0 from Python.

  Plus `TEST(Tracking, NeverDividesWhenStopAreaIsZero)`: `stop_area_fraction` 0 and area 0 give finite values with `linear_x == 0`. `CMakeLists.txt` and `package.xml` follow `vision_bot_perception`; the library has no ROS dependencies. Build, run, and see it fail (stub returns zeros).
- [ ] **Step 4: Implement `compute_tracking_cmd`**, step for step with the Python:
  - deadband (`|offset| < deadband` sets offset to 0.0);
  - `angular_z = -gain * offset`;
  - `area >= stop` gives `linear_x = 0`;
  - otherwise `proximity = max(0.2, 1 - area / stop)` and `linear_x = speed * (1 - min(1, |offset|)) * proximity`.

  Run all tests and see them pass, with no warnings.
- [ ] **Step 5: Commit and push.**

### Task 2: The node, launch selection, check_v1, CI, docs, then the Gate

Branch `v2/cpp-8-control-node` off Task 1.

**Interfaces:**
- Consumes: `compute_tracking_cmd`, `TrackingParams` (Task 1).
- Produces:
  - Component `vision_bot_control::MotorControlNode(const rclcpp::NodeOptions &)`, registered with `rclcpp_components_register_nodes`.
  - `rclcpp::Node::SharedPtr make_motor_control_node(const rclcpp::NodeOptions &)`.
  - Executable `motor_control_node` from `src/motor_control_main.cpp`, written like `vision_bot_perception/src/perception_main.cpp`.
  - Launch argument `control_impl`, default `python`, values `python` | `cpp`.

- [ ] **Step 1: Failing launch test** `test/test_motor_control_node.py`. The node starts with `lost_timeout_sec` 0.75. The test publishes detections and reads `/cmd_vel`:
  - `test_detection_drives_toward_target`: `[1, 0.5, 0, 0.01, 1]` gives a Twist matching the fixture case for (0.5, 0.01) to 1e-6.
  - `test_not_detected_publishes_nothing_then_watchdog_searches`: while sending `[0, ...]` at 10 Hz after a right-side detection, no Twist arrives for about 0.6 s. After that, Twists have `linear.x == 0` and `angular.z == +0.3` (last seen right is `+1 * 0.3`; the Python sign convention is `search_angular_speed * last_seen_direction`). The first search Twist arrives between 0.75 and 1.2 s after the last detection (Review Focus 2).
  - `test_disabled_publishes_nothing`: after `Bool(false)`, nothing on `/cmd_vel` for 1.5 s, even with detections arriving. After `Bool(true)`, Twists resume (Review Focus 3).
  - `test_malformed_detection_is_ignored` (Review Focus 1): a 3-element message produces no Twist, the process stays alive, and the next valid detection is answered.
  - `test_param_set_takes_effect`: setting `angular_gain` to 2.0 doubles `angular.z` for the same detection.
- [ ] **Step 2: Run it and see it fail** (no executable).
- [ ] **Step 3: Implement `MotorControlNode`** with the parameters and defaults from Global Constraints:
  - **Subscriptions:** on the two topics, depth 10.
  - **Watchdog:** a wall timer (`create_wall_timer(100ms)`).
  - **Last detection time:** `std::chrono::steady_clock::time_point`, initialized to the epoch, so the first watchdog tick sees "lost".
  - **Malformed input:** a message of size != 5 is ignored with `RCLCPP_WARN_THROTTLE` (5 s) naming the size.
  - **Logs:** the startup line is Python's plus ` (C++)`: `motor_control_node up, publishing on /cmd_vel (C++)`. The enable and disable log lines are the same as Python's.
- [ ] **Step 4: Run it and see it pass:** `colcon test --packages-select vision_bot_control` gives gtests and launch test PASS.
- [ ] **Step 5: Launch selection.** In `sim_launch.py`, `nav_sim.launch.py` and `hardware_launch.py`, declare `control_impl` (default `python`) and set the motor_control `Node`'s `package` with the same `PythonExpression` pattern as `perception_impl`, choosing `vision_bot_control` for `cpp`.
  - Extend `test_perception_impl_launch.py`, parametrized by argument: (`perception_impl`, `perception_node`, `vision_bot_perception`, default `cpp`) and (`control_impl`, `motor_control_node`, `vision_bot_control`, default `python`).
  - Run the test against the unchanged launch files and see it fail, then against the edited ones and see it pass.
- [ ] **Step 6: CI.** Add `vision_bot_control` to the colcon build and test lines, and lint `scripts/make_control_fixtures.py` and `vision_bot_control/test`.
- [ ] **Step 7: Verify in Docker:**
  - `check_v1.sh` passes on the defaults.
  - `V1_LAUNCH_ARGS="control_impl:=cpp"` passes.
  - `V1_LAUNCH_ARGS="perception_impl:=python control_impl:=python"` passes.
  - Live, with both nodes in C++ (`perception_impl:=cpp control_impl:=cpp`), the red-box demo stops at area fraction >= 0.15 and holds, and `line_follow_launch.py` completes a lap. These are the same criteria as the Gate 3 table in `docs/INTERVIEW_NOTES.md`; poll the rover with `gz model` and retry on timeout.
- [ ] **Step 8: Docs.**
  - **README:** `vision_bot_control` in the repo layout, and a line in the v2 section that the whole v1 pipeline can run in C++.
  - **DECISIONS #85 onward:** the package, the malformed-input difference, steady_clock, and the default pending the gate.
  - **Gate notes:** go in `docs/INTERVIEW_NOTES.md`.
- [ ] **Step 9: Commit and push, then check CI is green.**
- [ ] **Gate:** report the tests, the three check_v1 runs, the live runs and the CI run ID. Ask whether to make `control_impl` default to `cpp`, and whether to add this branch to PR #4 or open a separate PR. Stop.

## Not in this plan
No latency benchmark: the control law is a few floating-point operations per message, so the language makes no measurable difference to the robot (unlike perception, where the per-frame work is larger). The launch test's timing checks cover the watchdog.
