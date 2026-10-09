# vision_bot v2: Dependencies

Everything installs through `docker/Dockerfile`; rebuild with `docker compose -f docker/docker-compose.yml build`. Nothing is installed by hand in a running container.

## Base
- Image: `osrf/ros:humble-desktop-full` (Ubuntu 22.04.5, ROS 2 Humble, Python 3.10.12). Includes `rviz2`, `tf2_tools`, `ament_cmake_gtest`, Gazebo Classic 11 (`gazebo`, `gazebo_ros`, `gazebo_plugins`), and Gazebo Sim 6 / `ros_gz_*` (unused; the project is on Classic).
- v1 layer (unchanged): `gazebo-ros-pkgs`, `xacro`, `robot-state-publisher`, `cv-bridge`, `image-transport`, `teleop-twist-keyboard`, `rqt-image-view`, `rosbridge-suite`, `web-video-server`, colcon, OpenCV, pip, xvfb.

## v2 layer (added in Task 0)
Each name was checked with `apt-cache policy ros-humble-<pkg>` before it went into the Dockerfile.

| apt package | Provides | Installed version (2026-10-08) |
|---|---|---|
| `ros-humble-slam-toolbox` | `slam_toolbox` (online async SLAM) | 2.6.10-1jammy.20260908.014627 |
| `ros-humble-navigation2` | Nav2 stack, incl. `nav2_amcl`, `nav2_map_server`, `nav2_costmap_2d` | 1.1.20-1jammy.20260908.015443 |
| `ros-humble-nav2-bringup` | reference launch files and params (`nav2_params.yaml`, multi-robot launches) | 1.1.20-1jammy.20260908.021125 |
| `ros-humble-robot-localization` | `ekf_node` | 3.5.4-1jammy.20260908.012128 |
| `ros-humble-twist-mux` | `twist_mux` (uses `geometry_msgs/msg/Twist`) | 4.3.0-1jammy.20260907.224309 |

Pulled in or already present (versions in the rebuilt image):

| Package | Version |
|---|---|
| `ros-humble-nav2-amcl` | 1.1.20-1jammy.20260908.000809 |
| `ros-humble-nav2-map-server` | 1.1.20-1jammy.20260908.000810 |
| `ros-humble-nav2-costmap-2d` | 1.1.20-1jammy.20260908.002855 |
| `ros-humble-rviz2` | 11.2.28-1jammy.20260804.222726 |
| `ros-humble-tf2-tools` | 0.25.22-1jammy.20260804.195304 |
| `ros-humble-gazebo-ros` | 3.9.0-1jammy.20260907.222900 |
| `ros-humble-gazebo-plugins` | 3.9.0-1jammy.20260907.231520 |
| `ros-humble-ament-cmake-gtest` | 1.3.14-1jammy.20260226.013944 |
| `gazebo` (Classic) | 11.10.2+dfsg-1 |

Verified in the rebuilt image with `ros2 pkg list`: `slam_toolbox`, `nav2_bringup`, `nav2_amcl`, `nav2_map_server`, `nav2_costmap_2d`, `robot_localization`, `twist_mux`.

## Reference files the plan says to copy from (not write from memory)
- `/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml` (Task 3)
- `/opt/ros/humble/share/nav2_bringup/launch/{bringup,localization,navigation,slam}_launch.py` (Tasks 2, 3)
- `/opt/ros/humble/share/nav2_bringup/launch/{unique,cloned}_multi_tb3_simulation_launch.py` and `params/nav2_multirobot_params_*.yaml` (Task 5)

## Added later
Per-package dependencies are declared in each new package's `package.xml` as it is created (Tasks 1, 4, 6). Dashboard test tooling is added in Task 7.
