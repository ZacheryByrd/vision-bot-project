# vision_bot -- Vision-Guided Rover

Read **`PROJECT_PLAN.md`** first -- it has the full architecture rationale,
week-by-week roadmap, hardware parts list (optional), and resume framing.

This repo is a real, runnable starting point, not pseudocode:

```
ros2_ws/src/vision_bot/   ROS2 package: perception_node, motor_control_node,
                           gpio_motor_driver, Gazebo launch + world + URDF
dashboard/                 Next.js live-feed + status dashboard
PROJECT_PLAN.md            Full plan: roadmap, architecture, parts list
```

## Quickstart (simulation)

Requires ROS2 (Humble or Jazzy) and Gazebo installed -- easiest via the
official ROS2 Docker image or a native Ubuntu install; see
https://docs.ros.org/en/humble/Installation.html if you need to set that up.

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch vision_bot sim_launch.py
```

In another terminal, sanity-check topics before trusting the vision pipeline:

```bash
ros2 topic list
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Quickstart (dashboard)

```bash
cd dashboard
npm install
cp .env.local.example .env.local   # point at your rosbridge/web_video_server host
npm run dev
```

Requires `rosbridge_suite` and `web_video_server` running alongside the
ROS2 nodes -- see `dashboard/README.md` for exact commands.

## Status

Scaffold only -- nodes, launch files, and dashboard are wired up and should
run, but the vision detector defaults to a naive HSV color threshold and
the control gains are unTuned. Follow the week-by-week plan in
`PROJECT_PLAN.md` to take this from "boots up" to "reliably follows a
target."
