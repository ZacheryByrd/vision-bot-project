# vision-bot dashboard

Live camera feed + robot status, streamed from ROS2 into a Next.js page.
Reuses the same Next.js patterns as Shiftly -- the new part here is the
ROS2 <-> browser bridge, not the frontend itself.

## How the data gets from ROS2 to the browser

ROS2 topics aren't directly reachable from a browser, so two small
bridge pieces run alongside the robot's own nodes -- both are started
together by `ros2 launch vision_bot bridge_launch.py` (see the top-level
README's Quickstart for the full sequence in this repo's Docker setup):

1. **`rosbridge_suite`** -- exposes ROS2 topics over a WebSocket
   (`ws://<robot-host>:9090`, published as `9091` on the Windows host in
   this repo's `docker-compose.yml` -- see the comment there for why).
2. **`web_video_server`** -- serves `/vision_bot/debug_image` (the
   annotated frame from `perception_node`) as MJPEG over plain HTTP,
   which an `<img>` tag can point at directly without any JS decoding.
   (`:8080` in the container, published as `8081` on the Windows host.)

With both running, this Next.js app:
- Points an `<img>` tag at `http://<robot-host>:8080/stream?topic=/vision_bot/debug_image`
  for the live annotated feed.
- Opens a `roslibjs` WebSocket connection to `ws://<robot-host>:9090` and
  subscribes to `/vision_bot/detection` and `/cmd_vel` to render the
  status panel (detected y/n, offset, current velocity) in real time.

## Local dev

```
npm install
npm run dev
```

Set `NEXT_PUBLIC_ROS_WS_URL` and `NEXT_PUBLIC_VIDEO_STREAM_URL` (see
`.env.local.example`) to point at wherever rosbridge/web_video_server
are actually running -- your Pi's IP on the local network, or
`localhost` if you're running everything on one dev machine with Gazebo.

## What's scaffolded here vs. what you build

Scaffolded: the ROS connection hook, the video feed component, the
status panel component, a manual/autonomous control panel toggle
(publishes `std_msgs/Bool` to `/vision_bot/autonomous_enabled`), and
the page layout. Left for you: styling polish, and a connection-lost/
reconnect state -- right now `connected` just goes false and stays
there if the WebSocket drops, no retry.
