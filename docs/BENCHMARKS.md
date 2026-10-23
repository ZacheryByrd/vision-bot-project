# Reproducing the v2 numbers

Every number in the README's v2 section comes from one of the scripts below, and each run writes its raw data (CSV) and a markdown summary to `benchmarks/results/` with a timestamp. A summary records the git commit it ran on; "(+ uncommitted changes)" means the working tree differed from that commit when it ran (the configuration under test is named in the summary's "Configuration" line).

All runs happen inside the vision_bot container, headless:

```bash
# from the repo root on the host
docker compose -f docker/docker-compose.yml run --rm ros bash
# inside the container
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash
cd /workspace/vision-bot-project/benchmarks
```

Start the simulation in the background (`xvfb-run -a ros2 launch ... gui:=false &`) or in a second shell, wait for it to come up (for Nav2, until the log shows `Managed nodes are active`), then run the script. One simulation per container: a second Gazebo in the same container needs its own `GAZEBO_MASTER_URI`.

**Check before trusting a run.** Every gate run also counted `CRITICAL FAILURE|Resetting managed nodes` in the launch log and required 0. Nav2's servers send a heartbeat every 0.1 s and are shut down after 4 s without one, so a machine that sleeps or stalls (memory pressure) mid-run produces failures that are not about the configuration. Two such runs were discarded during the project (DECISIONS #59, #72).

## 1. Odometry drift: raw wheel odometry vs the EKF
| | |
|---|---|
| Result | `odom_drift_20261009_043714.{md,csv,png}` (commit `30524e4`) |
| Simulation | `ros2 launch vision_bot_nav nav_sim.launch.py gui:=false` |
| Command | `python3 odom_drift.py --label "encoder odometry, no added noise"` |
| What it does | Drives the `drift_loop` route (two laps of a rectangle, about 22 m) by script, steering from ground truth, and records raw odometry, the EKF output and ground truth at 10 Hz of sim time. Each path is taken relative to its own start pose and scored with `metrics.position_errors`. |

## 2. Nav2 goal tour (one robot)
| | |
|---|---|
| Results | baseline `nav_goals_20261010_003755` (commit `e8e7bc1` + the stock-defaults configuration named in the file); tuned `nav_goals_20261010_052224` (commit `d637cbe`) |
| Simulation | `ros2 launch vision_bot_nav nav2.launch.py gui:=false` |
| Command | `python3 nav_goals.py --label "<configuration under test>"` |
| What it does | Sends the 11 goals in `goals.yaml` in order through `NavigateToPose` (180 s of sim time each) and records success, time, path length, recoveries, and the final error by ground truth. The tuning between the two runs is in `docs/TUNING.md` and DECISIONS #35 and #36. |

## 3. Goal tours with two robots
| | |
|---|---|
| Results | `nav_goals_robot1_20261011_234057` (commit `5a8f6da` + the namespaced benchmark, committed as `72836f7`), `nav_goals_robot2_20261011_235724` (commit `72836f7`) |
| Simulation | `ros2 launch vision_bot_nav multi_robot.launch.py gui:=false`, a fresh launch per tour |
| Commands | `python3 nav_goals.py --namespace robot1 --goals goals.yaml --label "..."`, then in a fresh launch `python3 nav_goals.py --namespace robot2 --goals goals_robot2.yaml --label "..."` |
| What it does | Each robot drives the tour while the other stays parked at its spawn pose. `goals_robot2.yaml` differs only in its last goal, robot2's own spawn pose. |

## 4. RobotAvoidanceLayer A/B
| | |
|---|---|
| Results | against a scripted stand-in robot: `plugin_ab_20261010_065758` (commit `00956ac`); with two real robots: `plugin_ab_2robots_20261013_030935` (commit `b7cd0ff`) |
| Simulation | stand-in: `nav2.launch.py gui:=false` (the script spawns the stand-in box); two robots: `multi_robot.launch.py gui:=false` |
| Commands | `python3 plugin_ab.py --per-scenario 10 --label "..."`; `python3 plugin_ab.py --per-scenario 10 --robot1 robot1 --robot2 robot2 --label "..."` |
| What it does | Two scenarios (a robot parked in a doorway; a robot crossing the path), each run with the layer off and on with identical seeded placements and timing, condition order alternating. Robots are teleported to their start poses and AMCL is re-initialized every trial. "Truly arrived" means Nav2 success and within 0.30 m of the goal by ground truth. |
| Caveat | The contact column (robot centres closer than 0.20 m) was set for the stand-in box and undercounts contact between two real robots, whose bodies touch at 0.14 to 0.24 m depending on orientation; see DECISIONS #58. |

## 5. EKF replay regression
| | |
|---|---|
| Result | `replay_regression_baseline_20261016.txt` (5 replays), summarized in `benchmarks/baseline.json` |
| Recording | `bash /workspace/vision-bot-project/benchmarks/record_regression_bag.sh` (starts `nav_sim.launch.py` itself, drives `drift_loop`, writes `benchmarks/bags/drift_loop`, a 3.4 MB zstd bag of `/clock`, `/odom`, `/imu`, `/scan`, `/tf_static`, `/ground_truth/pose`) |
| Command | `python3 replay_regression.py --bag bags/drift_loop` (no simulation needed; about 165 s) |
| What it does | Plays the bag through a fresh `ekf_node` using `ekf.yaml` from the repo and scores the EKF output against the bag's ground truth like section 1. Exits 1 if the mean error is above `threshold_m` in `baseline.json` (0.08 m: the 0.0608 m baseline plus a 0.02 m margin, about 30 times the spread between replays). CI runs it on every push to a `v2/*` branch. |

## 6. perception_node, Python vs C++
| | |
|---|---|
| Results | HSV: `perception_impl_hsv_20261022_200811.{md,csv}`; DNN: `perception_impl_dnn_20261022_225800.{md,csv}` (both commit `b39f811`) |
| Capture (simulation running) | HSV: `ros2 launch vision_bot sim_launch.py gui:=false`, then `python3 perception_impl.py capture --out /tmp/frames_hsv.npz`. DNN: `ros2 launch vision_bot dnn_follow_launch.py gui:=false`, then `python3 perception_impl.py capture --out /tmp/frames_dnn.npz`. Stop the simulation afterwards. |
| Command (simulation stopped) | `python3 perception_impl.py run --frames /tmp/frames_hsv.npz --backend hsv`, and the same with `frames_dnn.npz --backend dnn` |
| What it does | `capture` keeps every 3rd camera frame (200 frames, 640x480 `rgb8`, as Gazebo publishes them). `run` starts each implementation in turn as its own process on `/bench/image`, alternating the order over 3 rounds. Latency is lockstep: publish one frame, wait for its detection, timed in the benchmark process (10 warm-up frames per start are not counted). CPU is the node process's `utime+stime` from `/proc/<pid>/stat` while the frames stream at 30 Hz for 20 s. Parity compares the two implementations' detections frame by frame. |
| Why the simulation is stopped | With Gazebo running, it and the simulation's own perception node take several cores (load average 16 during the DNN capture), and the measurement would mostly reflect that contention. The frames are the same simulation frames either way. |

## Not in benchmarks/results
Gate checks that are pass/fail rather than measurements (the fleet bridge's outage and token tests, the dashboard checks, the map check against the world file) are recorded with their evidence in `docs/INTERVIEW_NOTES.md` under each gate, not as benchmark numbers.
