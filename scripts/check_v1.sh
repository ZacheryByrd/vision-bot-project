#!/usr/bin/env bash
# v1 regression check (v2 spec invariant 4: "v1 is never broken").
#
# Run inside the dev container (from the repo root on the host):
#   docker compose -f docker/docker-compose.yml run --rm ros \
#       /workspace/vision-bot-project/scripts/check_v1.sh
#
# What it does:
#   1. Builds the colcon workspace from scratch into a temp directory, so it
#      never touches (or depends on) ros2_ws/build, install, or log.
#   2. Launches the v1 sim (sim_launch.py) headless under xvfb.
#   3. Asserts the v1 nodes appear in `ros2 node list` and the camera topic
#      publishes (`ros2 topic hz` reports a rate).
#   4. Shuts everything down.
# Exit 0 = pass, 1 = fail.
#
# Overrides (optional): V1_NODES (space-separated node names), V1_CAMERA_TOPIC,
# V1_TIMEOUT_S (seconds to wait for the nodes).

V1_NODES=${V1_NODES:-"perception_node motor_control_node"}
V1_CAMERA_TOPIC=${V1_CAMERA_TOPIC:-/camera/image_raw}
V1_TIMEOUT_S=${V1_TIMEOUT_S:-90}

REPO=$(cd "$(dirname "$0")/.." && pwd)
OUT=$(mktemp -d /tmp/check_v1.XXXXXX)
LAUNCH_PID=""

# Keep this check's DDS traffic inside this container, so a sim running in
# another container on the same Docker network can't produce a false pass.
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

log() { echo "[check_v1] $*"; }

cleanup() {
    if [ -n "$LAUNCH_PID" ]; then
        kill -INT -- "-$LAUNCH_PID" 2>/dev/null
        for _ in $(seq 1 10); do
            kill -0 "$LAUNCH_PID" 2>/dev/null || break
            sleep 1
        done
        kill -KILL -- "-$LAUNCH_PID" 2>/dev/null
        wait "$LAUNCH_PID" 2>/dev/null
    fi
    pkill -f gzserver 2>/dev/null
    ros2 daemon stop >/dev/null 2>&1
}
trap cleanup EXIT

fail() {
    log "FAIL: $*"
    log "launch log tail ($OUT/launch.log):"
    tail -n 20 "$OUT/launch.log" 2>/dev/null
    exit 1
}

# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

log "building $REPO/ros2_ws/src into $OUT"
if ! colcon --log-base "$OUT/log" build \
        --base-paths "$REPO/ros2_ws/src" \
        --build-base "$OUT/build" \
        --install-base "$OUT/install" > "$OUT/build.log" 2>&1; then
    tail -n 30 "$OUT/build.log"
    fail "colcon build"
fi
log "build ok: $(grep -E '^Summary' "$OUT/build.log")"

# shellcheck disable=SC1091
source "$OUT/install/setup.bash"

log "launching sim_launch.py headless"
setsid xvfb-run -a ros2 launch vision_bot sim_launch.py gui:=false \
    > "$OUT/launch.log" 2>&1 &
LAUNCH_PID=$!

missing="$V1_NODES"
for _ in $(seq 1 "$V1_TIMEOUT_S"); do
    nodes=$(ros2 node list 2>/dev/null)
    missing=""
    for n in $V1_NODES; do
        echo "$nodes" | grep -qx "/$n" || missing="$missing $n"
    done
    [ -z "$missing" ] && break
    sleep 1
done
[ -z "$missing" ] || fail "nodes not up after ${V1_TIMEOUT_S}s:$missing"
log "nodes up: $V1_NODES"

# The nodes start immediately, but the camera only publishes once Gazebo has
# spawned the robot, which can take a while on a loaded machine. Keep
# sampling for up to V1_TIMEOUT_S instead of giving it one 10 s window.
rate=""
deadline=$((SECONDS + V1_TIMEOUT_S))
while [ -z "$rate" ] && [ "$SECONDS" -lt "$deadline" ]; do
    rate=$(timeout 10 ros2 topic hz "$V1_CAMERA_TOPIC" 2>/dev/null \
        | awk '/average rate:/ {r=$3} END {print r}')
done
[ -n "$rate" ] || fail "no messages on $V1_CAMERA_TOPIC within ${V1_TIMEOUT_S}s"
log "$V1_CAMERA_TOPIC publishing at ${rate} Hz"

log "PASS"
exit 0
