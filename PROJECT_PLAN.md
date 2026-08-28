# Vision-Guided Rover — Project Plan

**Working name:** `vision_bot`
**Stack:** ROS2 (Humble/Jazzy), Python, OpenCV, Gazebo, Next.js
**Goal:** Close the two unproven skill gaps on the resume — ROS2 and Computer Vision/OpenCV — with a project that demonstrates real multi-node robotics architecture, not a single script.

---

## 1. Why this shape

Two nodes talking over topics is the actual thing robotics job postings screen for: can you decompose a system into perception and control, and get them communicating asynchronously in real time. A single monolithic `camera_to_motors.py` script does not demonstrate that, even if it technically works. So the non-negotiable architectural requirement is:

- `perception_node` — owns the camera, runs OpenCV, publishes a structured detection/tracking message.
- `motor_control_node` — subscribes to that message, owns motor/velocity output, publishes `/cmd_vel` (or drives GPIO directly on hardware).
- Nothing about vision logic lives in the control node, and nothing about motor logic lives in the perception node. That separation is the point — it's what a reviewer skimming the repo in 30 seconds should see immediately from the file layout.

## 2. Sim-first, hardware-later strategy

Build and validate entirely in Gazebo before touching hardware. This is standard practice in the industry (not a shortcut), it removes cost/parts-lead-time as a blocker to starting this week, and the same ROS2 nodes should run against either the simulated camera/diff-drive topics or the real Raspberry Pi camera/motors with only the launch file swapped — that portability is itself worth stating on the resume ("nodes are hardware-agnostic; validated in simulation and deployable to physical hardware unchanged").

Recommended sequence:

1. **Week 1 — Sim skeleton.** Get a diff-drive robot spawned in Gazebo with a camera sensor, confirm `/camera/image_raw` and `/cmd_vel` topics exist and are controllable manually (`teleop_twist_keyboard`).
2. **Week 2 — Perception node.** Subscribe to the camera topic, implement line-following (color/contour threshold) or object detection (color blob first, then optionally a lightweight model like a Haar cascade or MobileNet-SSD via OpenCV DNN). Publish a custom or std message with detection center/offset and confidence.
3. **Week 3 — Control node.** Subscribe to perception output, implement a simple proportional controller that turns detection offset into `/cmd_vel` (angular.z proportional to horizontal offset, linear.x reduced when off-center or object lost). Get the rover reliably following a line or tracking an object around the Gazebo world.
4. **Week 4 — Dashboard.** Stand up `rosbridge_suite` (websocket bridge) and a Next.js page that subscribes to the camera topic (via `web_video_server` or MJPEG) and a status topic, rendering live feed + node health + detection overlay.
5. **Week 5 (optional) — Hardware.** Port to a Raspberry Pi + camera + L298N motor driver + 2 DC motors chassis. Swap the sim launch file for a hardware launch file; nodes themselves shouldn't need logic changes, only topic remaps/config.
6. **Week 6 (optional stretch) — Add a second behavior** (obstacle avoidance via a cheap ultrasonic/IR sensor, or upgrade detection to a small trained model) to show the architecture generalizes, not just a one-off demo.

Weeks 5–6 are explicitly optional — the project is complete and resume-ready after week 4 even if hardware never gets bought.

## 3. Repo layout

```
vision_bot/
├── ros2_ws/
│   └── src/
│       └── vision_bot/
│           ├── vision_bot/
│           │   ├── __init__.py
│           │   ├── perception_node.py      # camera in, detection message out
│           │   └── motor_control_node.py   # detection in, /cmd_vel out
│           ├── launch/
│           │   ├── sim_launch.py           # Gazebo world + nodes
│           │   └── hardware_launch.py      # Pi camera + GPIO motor driver + nodes
│           ├── description/
│           │   └── vision_bot.urdf.xacro   # diff-drive robot model for Gazebo
│           ├── worlds/
│           │   └── track.world             # simple Gazebo world with a line/objects
│           ├── resource/vision_bot
│           ├── package.xml
│           ├── setup.py
│           └── setup.cfg
├── dashboard/                              # Next.js live-feed + status UI
└── docs/
    └── PROJECT_PLAN.md (this file)
```

## 4. Hardware option (if you go physical)

Not required — Gazebo alone is a complete, legitimate project. If you want physical hardware for a demo video (which does meaningfully strengthen a portfolio/interview story), a minimal cheap build:

| Part | Notes | Approx. cost |
|---|---|---|
| Raspberry Pi 4 (2GB+) | Or a Pi 3B+ if you already own one | $35–55 |
| Raspberry Pi Camera Module or USB webcam | USB webcam is simpler to get working fast | $10–25 |
| 2WD/4WD robot chassis kit w/ DC motors | Widely sold as "smart car chassis kit" | $15–25 |
| L298N or TB6612FNG motor driver | TB6612 is more efficient, either is fine | $5–8 |
| Battery pack (18650s or AA holder) | Separate from Pi power | $10–15 |
| Jumper wires, breadboard | | $5 |

Total: roughly $80–130. This is genuinely optional — call it out explicitly as a stretch goal, not a requirement, when you talk about this project.

## 5. Dashboard integration (ties back to Shiftly stack)

- `rosbridge_suite` exposes ROS2 topics over a WebSocket.
- `web_video_server` (or a small custom node re-publishing JPEG frames) serves the camera feed over HTTP/MJPEG.
- Next.js frontend (reuse patterns from Shiftly) connects via `roslibjs` to subscribe to the detection/status topic and render an `<img>` tag pointed at the MJPEG stream, plus a status panel (node alive/dead, current linear/angular velocity, last detection confidence).
- This is what turns the project from "an embedded/robotics hobby build" into a full-stack project — real-time data pipeline from a physical/simulated sensor through ROS2 through a websocket into a React UI.

## 6. Resume/portfolio framing (once built)

Suggested bullet shape once the perception + control loop is working in sim (fill in specifics once real numbers exist — don't pre-write false metrics):

- "Built a vision-guided rover in ROS2, decomposing perception (OpenCV-based object/line detection) and motor control into independent nodes communicating over pub/sub topics; validated in Gazebo simulation [and deployed to a Raspberry Pi rover]."
- "Built a real-time Next.js dashboard streaming live camera feed and robot telemetry from ROS2 via rosbridge/WebSocket."

Do not add these until the corresponding milestone is actually done — the whole point of this project is to make the resume claims true.

## 7. Definition of done (minimum bar to call this "shipped")

1. `perception_node` and `motor_control_node` run as separate ROS2 nodes and are visible as separate processes/nodes in `ros2 node list`.
2. Rover (sim or real) follows a line or tracks an object for at least a full lap/loop without manual intervention.
3. Dashboard shows live video + at least one real-time status value, not a static screenshot.
4. README with a GIF/video demo, architecture diagram, and "how to run" instructions — this is what a recruiter or interviewer will actually look at.

## 8. Immediate next action

Everything in `ros2_ws/src/vision_bot/` in the delivered scaffold is a real, runnable starting point (not pseudocode) — install ROS2 (Humble or Jazzy) + Gazebo locally or in a dev container, `colcon build`, and start on Week 1 (spawn the sim robot, drive it manually) before writing any vision code.
