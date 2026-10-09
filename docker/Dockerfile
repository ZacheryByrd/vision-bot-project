# ROS2 Humble + Gazebo Classic 11 dev environment for vision_bot.
# Windows has no native ROS2 install path worth using, so this container is
# the actual dev environment: build and run everything in here, edit files
# on the host (the ros2_ws bind mount in docker-compose.yml keeps them in sync).
FROM osrf/ros:humble-desktop-full

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt-image-view \
    ros-humble-rosbridge-suite \
    ros-humble-web-video-server \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-pip \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# v2 navigation stack: SLAM, EKF localization, Nav2 (includes map_server and
# AMCL), and the velocity mux. Separate layer so the v1 layer above stays
# cached. Names verified with `apt-cache policy ros-humble-<pkg>`; see
# docs/DEPENDENCIES.md. rviz2 and tf2_tools already ship with desktop-full.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization \
    ros-humble-twist-mux \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi" >> /root/.bashrc

WORKDIR /workspace/ros2_ws

CMD ["/bin/bash"]
