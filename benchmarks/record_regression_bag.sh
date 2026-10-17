#!/bin/bash
# Record the rosbag for benchmarks/replay_regression.py (run inside the
# vision_bot container, from the repo root):
#
#   docker compose -f docker/docker-compose.yml run --rm --no-deps -T ros \
#       bash /workspace/vision-bot-project/benchmarks/record_regression_bag.sh
#
# Starts nav_sim.launch.py headless, records the sim clock, wheel odometry,
# IMU, lidar, the static transforms and ground truth while drive_route.py
# drives the Gate 2 drift_loop route (two laps of a rectangle, about 22 m), and writes a
# zstd-compressed bag to benchmarks/bags/drift_loop. The live /tf is left out
# on purpose: on replay the EKF publishes odom -> base_link itself.
set -e
source /opt/ros/humble/setup.bash
export ROS_LOCALHOST_ONLY=1 PYTHONUNBUFFERED=1
REPO=/workspace/vision-bot-project
OUT=/tmp/record_bag; mkdir -p $OUT
BAG=$REPO/benchmarks/bags/drift_loop
colcon --log-base $OUT/log build --base-paths $REPO/ros2_ws/src --packages-up-to vision_bot_nav \
  --build-base $OUT/build --install-base $OUT/install > $OUT/build.log 2>&1
source $OUT/install/setup.bash

setsid xvfb-run -a ros2 launch vision_bot_nav nav_sim.launch.py gui:=false > $OUT/launch.log 2>&1 &
LP=$!
trap 'kill -INT -- -$LP 2>/dev/null; for i in $(seq 1 30); do pgrep -x gzserver > /dev/null || break; sleep 1; done; pkill -9 -x gzserver || true' EXIT
# Ready when the EKF (part of nav_sim.launch.py) publishes and ground truth flows.
for i in $(seq 1 36); do
  timeout 5 ros2 topic echo --once --qos-reliability best_effort /odometry/filtered nav_msgs/msg/Odometry > /dev/null 2>&1 \
    && timeout 5 ros2 topic echo --once --qos-reliability best_effort /ground_truth/pose nav_msgs/msg/Odometry > /dev/null 2>&1 \
    && break
  sleep 1
done
echo "sim up (attempt $i)"
sleep 3

rm -rf $BAG; mkdir -p "$(dirname $BAG)"
ros2 bag record -o $BAG --compression-mode file --compression-format zstd \
  /clock /odom /imu /scan /tf_static /ground_truth/pose > $OUT/record.log 2>&1 &
RP=$!
sleep 3
(cd $REPO/benchmarks && python3 drive_route.py --route drift_loop)
sleep 2
kill -INT $RP; wait $RP || true
ros2 bag info $BAG
