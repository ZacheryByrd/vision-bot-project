# perception_node in C++ Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A C++ (rclcpp + OpenCV C++) `perception_node` that is a drop-in replacement for the Python one: same node name, topics, message layout and parameters, the same results on the same frames, selectable at launch, and measured against the Python node.

**Architecture:** A new ament_cmake package `vision_bot_perception`. A ROS-free library (`detectors`) holds the HSV and DNN detectors, unit tested with gtest against reference outputs produced by the Python functions. An rclcpp component (`vision_bot_perception::PerceptionNode`) wraps it and is also built as a standalone `perception_node` executable. Launch files gain one argument, `perception_impl` (`python` | `cpp`), that picks the package. A benchmark script runs both implementations on the same frames.

**Tech Stack:** ROS 2 Humble, rclcpp, rclcpp_components, cv_bridge, OpenCV 4 (core, imgproc, dnn), gtest, launch_testing, Python 3 for fixtures and the benchmark.

**Spec:** the Python node itself (`ros2_ws/src/vision_bot/vision_bot/perception_node.py`, its module docstring and `test/test_perception_logic.py`) plus the requirements agreed in chat on 2026-10-18, restated under Global Constraints.

## Global Constraints

- The Python `perception_node` is not changed and stays the default until Gate 4 decides otherwise. v1 is never broken: `scripts/check_v1.sh` passes at every gate.
- Same interface as the Python node, exactly: node name `perception_node`; subscribes `image_topic` (default `/camera/image_raw`, `sensor_msgs/Image`); publishes `/vision_bot/detection` (`std_msgs/Float32MultiArray`, `[detected, offset_x, offset_y, area_fraction, confidence]`) and `/vision_bot/debug_image` (`sensor_msgs/Image`, `bgr8`, header copied from the input); QoS depth 10 on all three.
- Same parameters, names, types and defaults: `image_topic` "/camera/image_raw", `target_hue_low` 0, `target_hue_high` 10, `target_sat_low` 120, `target_val_low` 80, `min_contour_area` 200 (int), `publish_debug_image` true, `detector_backend` "hsv", `dnn_prototxt_path` "", `dnn_model_path` "", `dnn_target_class_id` 15, `dnn_confidence_threshold` 0.5 (double). Detection parameters are read on every frame, as the Python node does, so `ros2 param set` takes effect at once.
- C++17, `-Wall -Wextra -Wpedantic` with no warnings, matching `vision_bot_plugins`.
- Everything builds and runs in the Docker image (`docker/docker-compose.yml`, service `ros`); no host installs.
- One branch per task, `v2/cpp-N-name`, each branched off the previous; push after every commit; no Co-Authored-By trailer.
- Never fabricate numbers: benchmark output is committed raw, and every number in docs comes from a committed file.
- Gates: stop after Tasks 3 and 4 with a report, and append to `docs/INTERVIEW_NOTES.md` at each gate.

## Review Focus

1. **`blobFromImage` mean in the DNN path.** Python passes `127.5` as a bare number; C++ must subtract exactly what Python subtracts (one channel or three). The DNN reference fixture decides it; the gtest compares confidence and box to 1e-3.
2. **Non-`bgr8` input encodings** (`rgb8` from Gazebo's `R8G8B8` camera). Python's `imgmsg_to_cv2(..., "bgr8")` converts; C++ must too, or colors flip and HSV finds nothing. Pinned by the Task 3 launch test sending an `rgb8` frame.
3. **DNN backend with missing or wrong model paths.** Python raises at startup. C++ must fail loudly at startup too (log the paths and throw), never run and publish `detected=0` forever. Pinned by a Task 3 test.
4. **Ties and rounding.** Several contours of equal area: Python's `max` keeps the first; `std::max_element` must too. `int(cx)` truncates; `static_cast<int>` must be used, not rounding. Covered by the HSV fixtures (two equal boxes case).
5. **`publish_debug_image=false`.** No debug image is published, and no `cv_bridge` conversion is done for it. Pinned by a Task 3 test.

---

## File Structure

```
ros2_ws/src/vision_bot_perception/
  package.xml, CMakeLists.txt
  include/vision_bot_perception/detectors.hpp   Detection, HsvParams, detect_target, dnn_detect_target (no ROS)
  src/detectors.cpp
  src/perception_node.cpp                        rclcpp component PerceptionNode (ROS plumbing only)
  test/test_detectors.cpp                        gtest against the Python reference fixtures
  test/fixtures/*.png, reference.json            frames and the Python node's outputs for them
  test/test_perception_node.py                   launch_testing: topics, encodings, params, failure modes
scripts/make_perception_fixtures.py              writes the fixtures from the Python functions
benchmarks/perception_impl.py                    Python vs C++ on the same frames
```
Modified: `ros2_ws/src/vision_bot/launch/sim_launch.py`, `ros2_ws/src/vision_bot_nav/launch/nav_sim.launch.py`, `ros2_ws/src/vision_bot/launch/hardware_launch.py` (one launch argument each), `scripts/check_v1.sh` (`V1_LAUNCH_ARGS`), `.github/workflows/v2-ci.yml`, README, `docs/BENCHMARKS.md`, `docs/DECISIONS.md`, `docs/INTERVIEW_NOTES.md`.

---

### Task 1: Package skeleton, reference fixtures and the HSV detector

Branch `v2/cpp-1-hsv-detector` off `main`. Commit this plan first.

**Files:**
- Create: `ros2_ws/src/vision_bot_perception/{package.xml,CMakeLists.txt}`, `include/vision_bot_perception/detectors.hpp`, `src/detectors.cpp`, `test/test_detectors.cpp`, `test/fixtures/`
- Create: `scripts/make_perception_fixtures.py`, `scripts/tests/test_make_perception_fixtures.py`

**Interfaces:**
- Produces (namespace `vision_bot_perception`):
  ```cpp
  struct Detection { bool detected = false; double offset_x = 0, offset_y = 0, area_fraction = 0, confidence = 0; };
  struct HsvParams { int hue_low = 0, hue_high = 10, sat_low = 120, val_low = 80; double min_area = 200; };
  // debug may be nullptr (no drawing). When non-null it receives a copy of frame, annotated as Python does.
  Detection detect_target(const cv::Mat & bgr, const HsvParams & p, cv::Mat * debug);
  ```
- Fixture format, `test/fixtures/reference.json`: `{"hsv": [{"image": "<name>.png", "params": {hue_low, hue_high, sat_low, val_low, min_area}, "expected": [detected, offset_x, offset_y, area_fraction, confidence]}], "dnn": [...]}` (`dnn` filled in Task 2).

- [ ] **Step 1: Write `scripts/make_perception_fixtures.py`** with `make_hsv_cases() -> list[tuple[str, np.ndarray, dict]]` and `main(out_dir)`. It builds 320x240 frames on the grey (40, 40, 40) background with the test file's `draw_box`, covering: empty; centered 60 px red box; left, right, above; 20 px and 100 px; 5 px (below `min_area`); blue box; two equal 40 px red boxes at x=80 and x=240 (tie); a yellow box with `hue_low=25, hue_high=35` (the line-follow tuning). It writes each as PNG, runs the Python `detect_target` on the PNG read back from disk (so both languages see identical pixels), and writes `reference.json`. Imports `detect_target` from `ros2_ws/src/vision_bot` via `sys.path`, like `seed_fleet.py` does for fleet_bridge.
- [ ] **Step 2: Test the generator.** `scripts/tests/test_make_perception_fixtures.py::test_writes_cases_and_reference` runs `main(tmp_path)` and asserts every `image` in `reference.json` exists, there are at least 11 HSV cases, and the centered-box case has `expected[0] == 1.0`. Run in the container: `python3 -m pytest -q scripts/tests/test_make_perception_fixtures.py` → PASS.
- [ ] **Step 3: Generate and commit the fixtures:** `python3 scripts/make_perception_fixtures.py ros2_ws/src/vision_bot_perception/test/fixtures`. The PNGs are a few KB each.
- [ ] **Step 4: Write the failing gtest** `test_detectors.cpp`: `TEST(HsvReference, MatchesPythonOnEveryFixture)` loads `reference.json` with `nlohmann_json` (rosdep key `nlohmann-json-dev`), runs `detect_target` on each PNG with its params, and expects `detected` equal and each float within `1e-6` of Python. Also `TEST(HsvDetector, DebugFrameSameSizeAsInput)` and `TEST(HsvDetector, NullDebugDoesNotCrash)`. CMake passes `FIXTURE_DIR` (this package's `test/fixtures`) and `VISION_BOT_SRC_DIR` (`${CMAKE_CURRENT_SOURCE_DIR}/../vision_bot`, for the model files and the person photo) as compile definitions.
- [ ] **Step 5: Package files.** `package.xml` (format 3, ament_cmake): depend `rclcpp`, `rclcpp_components`, `sensor_msgs`, `std_msgs`, `cv_bridge`, `libopencv-dev`; test_depend `ament_cmake_gtest`, `nlohmann-json-dev`, `launch_testing_ament_cmake`, `launch_testing`, `launch_ros`. `CMakeLists.txt` builds `detectors` as a shared library linking `opencv_core opencv_imgproc opencv_dnn` and the gtest, following `vision_bot_plugins/CMakeLists.txt`.
- [ ] **Step 6: Run to see it fail:** `colcon build --packages-select vision_bot_perception && colcon test --packages-select vision_bot_perception && colcon test-result --verbose` → link error or failures (no `detect_target` yet).
- [ ] **Step 7: Implement `detect_target`** in `src/detectors.cpp`, step for step with the Python: `cvtColor BGR2HSV`; `inRange(Scalar(hue_low, sat_low, val_low), Scalar(hue_high, 255, 255))`; `erode` then `dilate` with the default 3x3 kernel, 2 iterations each; `findContours(RETR_EXTERNAL, CHAIN_APPROX_SIMPLE)`; largest by `contourArea` with the first one winning ties; reject if `area < min_area`; offsets from the bounding-rect center normalized by half width/height; `area_fraction = min(1.0, area / (w*h))`; confidence 1.0. Debug drawing: green rect (2 px), red filled circle radius 5 at `(int)cx, (int)cy`, blue center line 1 px.
- [ ] **Step 8: Run to see it pass:** same command → all tests PASS, no compiler warnings in `colcon build` output.
- [ ] **Step 9: Commit and push** (`git add ros2_ws/src/vision_bot_perception scripts/make_perception_fixtures.py scripts/tests/test_make_perception_fixtures.py docs/superpowers/plans/2026-10-18-perception-cpp.md`).

### Task 2: The DNN detector

Branch `v2/cpp-2-dnn-detector` off Task 1.

**Files:**
- Modify: `include/vision_bot_perception/detectors.hpp`, `src/detectors.cpp`, `test/test_detectors.cpp`, `scripts/make_perception_fixtures.py`, its test, `test/fixtures/reference.json`

**Interfaces:**
- Consumes: `Detection`, fixture format (Task 1).
- Produces:
  ```cpp
  Detection dnn_detect_target(const cv::Mat & bgr, cv::dnn::Net & net, int target_class_id,
                              double confidence_threshold, cv::Mat * debug);
  ```
  Model files stay in `vision_bot/models/dnn/` (not copied); tests and the node find them through that package's share directory or the source path.

- [ ] **Step 1: Extend the generator** with `make_dnn_cases()`: the person photo (`person.png`) as is, the same photo at threshold 0.99 (expected no detection, matching the Python test), the photo with `target_class_id=7` (car: no detection), and a blank grey frame. Expected values come from the Python `dnn_detect_target`. Extend the generator test: at least 4 DNN cases, the person case detected.
- [ ] **Step 2: Regenerate the fixtures** and commit `reference.json` (the person PNG is referenced by path into `vision_bot`, not copied).
- [ ] **Step 3: Write the failing gtest** `TEST(DnnReference, MatchesPythonOnEveryFixture)`: loads the net once with `cv::dnn::readNetFromCaffe`, expects `detected` equal, confidence and offsets within `1e-3` of Python. `GTEST_SKIP()` only if the model files are missing, as the Python test skips.
- [ ] **Step 4: Run to see it fail.**
- [ ] **Step 5: Implement `dnn_detect_target`:** resize to 300x300, `blobFromImage(resized, 0.007843, Size(300, 300), mean, false, false)` with `mean` set to whatever reproduces Python (Review Focus 1: try `Scalar(127.5)` first and `Scalar(127.5, 127.5, 127.5)` if the fixture disagrees; record which in a comment); best detection of the target class at or above the threshold, first one winning ties; box scaled by `w, h`; `area_fraction` from the box; confidence is the model score; same debug drawing as HSV.
- [ ] **Step 6: Run to see it pass.**
- [ ] **Step 7: Commit and push.** Note in the commit message which mean reproduced Python and why.

### Task 3: The rclcpp node, launch selection, check_v1 and CI — then Gate 3

Branch `v2/cpp-3-node` off Task 2.

**Files:**
- Create: `src/perception_node.cpp`, `test/test_perception_node.py`
- Modify: `CMakeLists.txt`, `package.xml`, `sim_launch.py`, `nav_sim.launch.py`, `hardware_launch.py`, `scripts/check_v1.sh`, `.github/workflows/v2-ci.yml`

**Interfaces:**
- Consumes: `detect_target`, `dnn_detect_target`, `HsvParams`, `Detection`.
- Produces: component class `vision_bot_perception::PerceptionNode(const rclcpp::NodeOptions &)`, registered with `rclcpp_components_register_node(perception_component PLUGIN "vision_bot_perception::PerceptionNode" EXECUTABLE perception_node)`, so `ros2 run vision_bot_perception perception_node` works and the component can be loaded into a container. Launch argument `perception_impl`, default `python`, values `python` | `cpp`.

- [ ] **Step 1: Write the failing launch test** `test/test_perception_node.py` (launch_testing, registered with `add_launch_test(... TIMEOUT 120)`), which starts the C++ node with `image_topic:=/test/image` and checks:
  - `test_centered_red_box_bgr8`: publish the centered-box fixture as `bgr8`; the next `/vision_bot/detection` equals the fixture's expected values within 1e-6, and a `/vision_bot/debug_image` arrives with the input's `header.stamp` and `bgr8` encoding.
  - `test_rgb8_input_is_converted` (Review Focus 2): the same frame sent as `rgb8` gives the same detection.
  - `test_param_set_takes_effect`: `ros2 param set`-equivalent (`SetParameters` service) `min_contour_area` to 100000; the same frame now gives `detected == 0.0`.
  - `test_no_debug_image_when_disabled` (Review Focus 5): a second node instance with `publish_debug_image:=false` and its own topics publishes detections and no debug image within 2 s.
  - `test_dnn_bad_model_path_fails_at_startup` (Review Focus 3): starting with `detector_backend:=dnn dnn_model_path:=/nonexistent` makes the process exit non-zero within 10 s with the path in its log.
- [ ] **Step 2: Run to see it fail** (no executable yet).
- [ ] **Step 3: Implement `PerceptionNode`** in `src/perception_node.cpp`: declare the parameters with the Global Constraints defaults; if `detector_backend == "dnn"` load the net in the constructor and throw `std::runtime_error` naming both paths if loading fails; subscription callback converts with `cv_bridge::toCvShare(msg, "bgr8")`, reads the detection parameters, calls the detector (passing a debug `cv::Mat*` only when `publish_debug_image` is true), publishes the 5-float array and the debug image with the input header. Log line at startup matching Python's: `perception_node up, subscribed to '<topic>', publishing detections on '/vision_bot/detection'` plus ` (C++)`.
- [ ] **Step 4: Run to see it pass:** `colcon test --packages-select vision_bot_perception` → gtests and launch test PASS.
- [ ] **Step 5: Launch selection.** In `sim_launch.py`, `nav_sim.launch.py` and `hardware_launch.py`, add `DeclareLaunchArgument("perception_impl", default_value="python", description=...)` and set the perception `Node`'s `package` to `PythonExpression(["'vision_bot_perception' if '", LaunchConfiguration("perception_impl"), "' == 'cpp' else 'vision_bot'"])`. Nothing else in those files changes; `dnn_follow_launch.py` and `line_follow_launch.py` pass the argument through by including `sim_launch.py` (verify with `ros2 launch vision_bot line_follow_launch.py --show-args`). Extend `ros2_ws/src/vision_bot_nav/test` with a test that `nav_sim.launch.py` declares `perception_impl` defaulting to `python`, following the existing launch-helper tests there.
- [ ] **Step 6: `check_v1.sh`:** add `V1_LAUNCH_ARGS` (default empty) appended to the `ros2 launch` line, documented in the header next to the other overrides.
- [ ] **Step 7: CI:** in `v2-ci.yml` add `vision_bot_perception` to `--packages-up-to` and to `colcon test --packages-select`, and add `ros-humble-cv-bridge` to the runtime apt line if rosdep does not bring it.
- [ ] **Step 8: Verify in Docker:**
  - `scripts/check_v1.sh` → PASS (Python default).
  - `V1_LAUNCH_ARGS="perception_impl:=cpp" scripts/check_v1.sh` → PASS, and the launch log shows the `(C++)` startup line.
  - Live parity in sim: `sim_launch.py perception_impl:=cpp gui:=false` drives the red-box demo to the stop distance, and `line_follow_launch.py perception_impl:=cpp` completes a lap (same pass criteria the v1 README records).
- [ ] **Step 9: Commit and push; CI green on the branch.**
- [ ] **Gate 3:** report (tests, both check_v1 runs, the live runs, CI run ID), append to `docs/INTERVIEW_NOTES.md`, stop.

### Task 4: Python vs C++ benchmark, docs — then Gate 4

Branch `v2/cpp-4-benchmark` off Task 3.

**Files:**
- Create: `benchmarks/perception_impl.py`, `benchmarks/tests/test_perception_impl.py`, `benchmarks/results/perception_impl_<timestamp>.{csv,md}`
- Modify: README (v2 results + a C++ line in the architecture section), `docs/BENCHMARKS.md` (section 6), `docs/DECISIONS.md`, `docs/INTERVIEW_NOTES.md`

**Interfaces:**
- Consumes: both `perception_node` executables, their `image_topic` parameter.
- Produces: `summarize(latencies_s: list[float]) -> dict` with keys `n, mean_ms, p50_ms, p95_ms, max_ms`; `cpu_percent(proc_stat_before, proc_stat_after, wall_s, clk_tck) -> float`.

- [ ] **Step 1: Test the pure helpers** in `benchmarks/tests/test_perception_impl.py`: `summarize([0.001, 0.002, 0.003, 0.004])` gives `n == 4`, `mean_ms == 2.5`, `p50_ms == 2.5`, `max_ms == 4.0`; `cpu_percent` with 50 ticks over 1 s at `clk_tck=100` gives `50.0`. Run → FAIL, implement, run → PASS.
- [ ] **Step 2: Write the benchmark.** It captures 200 frames from `/camera/image_raw` of a running sim into memory (for `--backend dnn`, from `dnn_follow_launch.py`'s world), then for each implementation in turn starts the node with `image_topic:=/bench/image` (order alternating over `--rounds 3`), and measures:
  - **latency:** lockstep, publish one frame, wait for its `/vision_bot/detection` (timeout 2 s), time with `time.perf_counter`; detections also recorded and compared between implementations frame by frame (count of frames whose `detected` differs, max float difference);
  - **CPU:** stream the frames at 30 Hz for 20 s and read the node process's own `/proc/<pid>/stat` utime+stime (the node process, not the `ros2 run` wrapper);
  - writes the raw per-frame CSV and a markdown summary with the commit and configuration, like the other benchmarks.
- [ ] **Step 3: Run it in Docker** for `--backend hsv` (track.world) and `--backend dnn`, with the launch-log check for `CRITICAL FAILURE` from `docs/BENCHMARKS.md`. Commit the raw results.
- [ ] **Step 4: Docs.** README: a results table row set (latency mean/p95, CPU, parity) taken from the committed summaries, and `perception_node` listed as available in Python and C++. `docs/BENCHMARKS.md` section 6 with the exact commands. DECISIONS: the mean finding (Task 2), the launch-argument design, the default decision pending Gate 4.
- [ ] **Step 5: Commit and push; CI green.**
- [ ] **Gate 4:** report the numbers from the committed files, and ask whether to make `cpp` the default `perception_impl` (one-line change per launch file plus a check_v1 run). Append to `docs/INTERVIEW_NOTES.md`, stop.

---

## After Gate 4

`motor_control_node` in C++ is the follow-up offered in chat; it gets its own plan once this one is merged.
