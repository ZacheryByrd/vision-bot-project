"""
bridge_launch.py
=================
Brings up the two pieces that let the Next.js dashboard talk to ROS2:
    - rosbridge_websocket: exposes ROS2 topics over a WebSocket (roslibjs
      on the dashboard side subscribes to /vision_bot/detection and /cmd_vel
      through this).
    - web_video_server: serves /vision_bot/debug_image as an MJPEG stream
      over plain HTTP, which the dashboard renders with a plain <img> tag.

Run this alongside sim_launch.py (separate terminal) -- it's independent
of whether the sim or hardware launch is driving the robot:
    ros2 launch vision_bot bridge_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
    )

    web_video_server = Node(
        package="web_video_server",
        executable="web_video_server",
        name="web_video_server",
        output="screen",
        parameters=[{"port": 8080}],
    )

    return LaunchDescription([rosbridge, web_video_server])
