# vision_bot v2: Fleet-Connected Autonomous Navigation (Design Spec)

**Date:** 2026-10-08
**Owner:** Zach
**Status:** Draft for review (not approved; nothing is to be built until approved)
**Related:** `vision_bot_v2_nav_plan.md` (detailed runbook for Claude Code; to be reconciled with this spec during planning)

---

## 1. Intent

**Outcome.** Extend the shipped vision_bot (v1) into an autonomous multi-robot navigation system that (a) makes Zach a stronger candidate for robotics software engineering roles and (b) teaches him how the navigation stack works.

**Stated by Zach:**
- Goals are landing a robotics job and learning the stack.
- v2 is built on top of the existing vision_bot repo, robot, and dashboard.
- Time is not a constraint (near full-time availability).
- Claude Code builds everything; Zach reviews at each gate.
- Scope is the full plan: SLAM, EKF, Nav2, custom C++ plugin, two robots, fleet bridge, dashboard map, CI.

**Assumptions (to be confirmed or corrected during review):**
- Simulation only; hardware is optional and out of scope.
- The project must be demo-ready for job applications, not just functional.
- Because Claude Code writes the code, interview readiness depends on documentation Zach reads and understands (section 7), not on authorship.

**Success criteria:** see section 9.

## 2. Scope

### In scope
1. Lidar and IMU on the robot model; a new walled navigation world.
2. `twist_mux` to arbitrate velocity commands between teleop, v1 line-following, and Nav2.
3. SLAM (`slam_toolbox`), a saved map, and an EKF (`robot_localization`) fusing odometry and IMU, measured against ground truth.
4. Nav2 with AMCL on the saved map, tuned and benchmarked.
5. A custom C++ Nav2 costmap layer (`RobotAvoidanceLayer`) that makes each robot avoid the others, with unit tests and an A/B benchmark.
6. A second robot; each robot runs in its own namespace with its own TF tree on a shared map.
7. A `fleet_bridge` ROS2 node per robot that authenticates to the existing fleet API, reports telemetry, receives goals, and reports status.
8. A dashboard map view with click-to-goal that routes through the fleet API.
9. Rosbag-replay regression tests and a GitHub Actions workflow.
10. README with architecture diagram, demo GIF, reproducible commands, and benchmark tables.

### Out of scope
Real hardware; a custom controller plugin (stretch only, on request); 3D or visual SLAM; more than two robots; changing v1 behavior.

## 3. Architecture

```
Fleet Platform (existing: FastAPI, PostgreSQL, Next.js, JWT/RBAC)
      ^ REST (WebSocket/SSE if available)          ^
      |                                            |
 fleet_bridge /robot1                         fleet_bridge /robot2
      | NavigateToPose, pose, status, battery      |
 /robot1: lidar, imu, odom -> EKF -> AMCL -> Nav2 -> twist_mux -> diff drive
 /robot2: same stack, own namespace, own TF tree, shared saved map
 v1 perception + motor_control remain, feeding twist_mux as one input
```

### New components

| Component | Purpose | Interface | Depends on |
|---|---|---|---|
| `vision_bot_nav` | Launch files, params, maps, worlds, RViz configs | Launch arguments (namespace, world, use_sim_time) | Nav2, slam_toolbox, robot_localization, twist_mux |
| `vision_bot_plugins` (C++) | `RobotAvoidanceLayer` costmap layer | Nav2 plugin (pluginlib); params: `robot_pose_topics`, `robot_radius`, `cost_scaling`, `max_pose_age_s`, `enabled` | nav2_costmap_2d |
| `fleet_bridge` (Python) | Connect one robot to the fleet API | Params: namespace, robot ID, API base URL; env vars for credentials | Fleet API, Nav2 action server |
| `benchmarks/` | Scripts that produce all reported numbers | CSV + markdown summaries in `benchmarks/results/` | Running stack |
| Dashboard map view | Show map, robot poses, accept click-to-goal | Fleet API (goals), fleet telemetry | Existing Next.js dashboard |

### Invariants (enforced at every gate)
1. **One owner of the final velocity command.** Every source publishes to its own topic; `twist_mux` selects (priority: teleop > line-follow > navigation) and publishes the only command the drive plugin consumes.
2. **One publisher per TF transform.** The EKF owns `odom -> base_link` (the drive plugin's TF output is disabled); AMCL owns `map -> odom` during navigation; slam_toolbox owns it during mapping, never both. Static frames come from `robot_state_publisher`.
3. **Sim time everywhere.** Every simulated node uses `use_sim_time: true`.
4. **v1 is never broken.** `scripts/check_v1.sh` runs at every gate. v1 changes are limited to launch-file remaps; any code change must be minimal and justified.
5. **No fabricated numbers.** Every reported metric comes from a script in `benchmarks/` with raw output committed.

## 4. Phases and gates

The work proceeds in order; Claude Code stops at each gate and reports. Zach approves before the next phase begins.

| Phase | Deliverable | Gate (must all pass) | Zach reviews |
|---|---|---|---|
| 0. Discovery | `docs/V2_DISCOVERY.md`; installed dependencies; `scripts/check_v1.sh` | Clean build; v1 check passes; distro, simulator stack, frames, topics, and fleet API endpoints recorded | Discovery doc; confirm simulator stack |
| 1. Sensors + mux | Lidar, IMU, new world, `twist_mux`, nav launch file | `/scan` and `/imu` at expected rates; scan matches walls in RViz; single TF parent per frame; teleop overrides line-follow; v1 check passes | Screenshot of scan over world |
| 2. SLAM + EKF | Saved map; EKF; ground-truth publisher; drift benchmark | Map matches world; no TF errors; drift benchmark ran with real numbers comparing raw odometry, EKF, and ground truth (with documented odometry noise/slip) | Map image; drift table |
| 3. Nav2 | AMCL, tuned costmaps, goal benchmark | Goal from RViz is reached; benchmark of at least 10 goals run (target 9 of 10; actual rate recorded either way); teleop still overrides Nav2 | Benchmark CSV; GIF of a run |
| 4. C++ plugin | `RobotAvoidanceLayer`, gtest tests, A/B benchmark | Builds cleanly; `colcon test` passes; Nav2 loads the layer; A/B results saved (at least 10 trials per condition, same goals and seeds) | A/B table (a null result is acceptable if reported honestly) |
| 5. Two robots + fleet | Namespaced stacks, `fleet_bridge`, dashboard map | Two independent stacks with no `cmd_vel` cross-talk; both localize and navigate; a dashboard click moves the chosen robot and status flows back; bridge survives API restart, expired token, and goal rejection; A/B benchmark rerun with two real robots | GIF of click-to-motion |
| 6. CI + docs | Rosbag replay test, GitHub Actions, README | CI green; a deliberately broken threshold turns CI red; a fresh clone reaches a working demo; every README number maps to a file in `benchmarks/results/` | README and CI run |

**Expected duration:** about 1.5 weeks best case with Claude Code working largely unattended. Phases 3 and 5 (tuning, namespacing) are the most likely to need rework, so plan for more.

## 5. Key design decisions

- **Same repo, new packages.** v1 packages stay untouched; v2 adds `vision_bot_nav`, `vision_bot_plugins`, and `fleet_bridge`.
- **Robot model:** a copy or extension of the v1 URDF in `vision_bot_nav` rather than edits to the v1 file.
- **Simulator variants:** Phase 0 determines Gazebo Classic vs the newer Gazebo from the repo; only the matching variant is implemented. If the repo is on Classic with a distro that does not support it, Claude Code stops and asks before any migration.
- **Nav2 params:** start from the installed distro's `nav2_bringup` defaults and diff; never write from memory.
- **Multi-robot namespacing:** follow the installed `nav2_bringup` multi-robot reference; each robot has its own namespaced TF tree and Nav2 instance, sharing one map.
- **Custom plugin choice:** `RobotAvoidanceLayer` (reads other robots' poses, adds decaying cost around them). Chosen because it uses the two-robot setup and is benchmarkable.
- **Fleet integration:** use the platform's real auth flow (JWT with refresh-token rotation) via a dedicated service account supplied by environment variables; goals flow dashboard -> fleet API -> bridge -> Nav2, so the whole path is exercised. If required endpoints are missing, Claude Code proposes the smallest addition to Zach first, then implements it with tests, RBAC, and migrations if needed.
- **Dashboard map:** static map image with metadata for the pixel-to-world transform (resolution, origin, flipped y-axis), unit tested with known points; the occupancy grid is not streamed continuously.
- **Battery:** simulated and clearly labeled as such.

## 6. Risks and mitigations

| Risk | Mitigation |
|---|---|
| Gazebo Classic vs newer Gazebo mismatch | Phase 0 detection; variant per stack; stop and ask on unsupported combination |
| Namespaced Nav2 / TF remap errors | Use the installed multi-robot reference; gate checks for independent stacks and no cross-talk |
| v1 zero-velocity output blocks Nav2 in `twist_mux` | Gate v1 output behind line-follow mode; document the fix |
| Perfect simulated odometry makes the EKF comparison meaningless | Add realistic noise or slip in Phase 2 and document parameters |
| Plugin shows no benefit | Report the true A/B result; method is the deliverable |
| Fleet API lacks needed endpoints | Propose minimal addition first; implement with tests and RBAC |
| Full Gazebo too heavy for CI | CI runs headless unit tests and rosbag replay only |
| Lidar produces no data in headless newer Gazebo | Verify Sensors system plugin and render backend in Phase 1 |

The runbook (`vision_bot_v2_nav_plan.md`, section 9) lists symptom-to-cause pairs for debugging.

## 7. Interview readiness (consequence of delegating all implementation)

Because Claude Code writes the code, Zach's ability to explain the project depends on documentation:
- Claude Code maintains `docs/INTERVIEW_NOTES.md` throughout, in plain English: what was built, why each choice was made, what broke and how it was diagnosed, and how each metric was measured.
- At each gate the report includes the new entries; Zach reads them before approving the next phase.
- Phase 6 adds a short Q&A section derived from the notes (for example, EKF configuration rationale, how AMCL recovers, what the costmap layer computes, how bridge auth refresh works).
- Supporting docs: `docs/ARCHITECTURE.md`, `docs/TUNING.md`, `docs/DECISIONS.md`, `docs/BENCHMARKS.md`.

## 8. Testing strategy

- **Unit tests:** costmap layer cost computation (gtest), `fleet_bridge` with mocked HTTP and mocked action client (pytest), dashboard pixel-to-world conversion with known points.
- **Integration benchmarks:** scripts in `benchmarks/` for odometry/EKF drift, goal success, and plugin A/B; raw results committed to `benchmarks/results/`.
- **Regression:** `scripts/check_v1.sh` at every gate; rosbag replay with a localization-error threshold in CI (threshold set from measured baseline plus a documented margin).
- **Proof tests can fail:** at Gate 6 a threshold is deliberately broken to confirm CI fails.

## 9. Definition of done

1. Two simulated robots map, localize, and navigate autonomously with Nav2 on a shared map.
2. The custom C++ costmap layer is built, unit tested, loaded by Nav2, and benchmarked against the stock configuration.
3. The EKF's performance versus raw odometry is measured against ground truth.
4. A goal clicked on the dashboard map flows through the fleet API and bridge to the right robot, and status flows back.
5. v1 line-following still works and is selectable through `twist_mux`.
6. CI runs build, unit tests, and the rosbag replay regression test, and is proven able to fail.
7. README includes the architecture diagram, demo GIF, reproducible commands, and benchmark numbers that each trace to a file in `benchmarks/results/`.
8. `docs/INTERVIEW_NOTES.md` is complete and Zach has read it.

Resume bullets are added only after the matching gate passes, using measured numbers.

## 10. Items resolved in Phase 0 (not open design questions)

These are facts to discover, not choices to make: ROS distro, Gazebo stack, existing frame and topic names, robot dimensions, dashboard connection details, fleet API endpoints, and whether the fleet repo is present in the workspace.
