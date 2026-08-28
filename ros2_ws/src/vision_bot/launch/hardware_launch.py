"""
hardware_launch.py
===================
Runs the exact same perception_node and motor_control_node as sim_launch.py,
but points perception at a real camera (v4l2_camera, works with any USB
webcam or the Pi camera via libcamera's v4l2 compat layer) and adds
gpio_motor_driver to actually turn the wheels.

Run with:
    ros2 launch vision_bot hardware_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name="camera",
        output="screen",
        parameters=[{"video_device": "/dev/video0", "image_size": [640, 480]}],
        remappings=[("/image_raw", "/camera/image_raw")],
    )

    perception = Node(
        package="vision_bot",
        executable="perception_node",
        name="perception_node",
        output="screen",
        parameters=[{"image_topic": "/camera/image_raw"}],
    )

    motor_control = Node(
        package="vision_bot",
        executable="motor_control_node",
        name="motor_control_node",
        output="screen",
        parameters=[{"base_linear_speed": 0.12}],  # a bit more conservative IRL
    )

    gpio_driver = Node(
        package="vision_bot",
        executable="gpio_motor_driver",
        name="gpio_motor_driver",
        output="screen",
    )

    return LaunchDescription([camera, perception, motor_control, gpio_driver])
