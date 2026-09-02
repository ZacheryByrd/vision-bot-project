import os
from glob import glob
from setuptools import find_packages, setup

package_name = "vision_bot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "description"), glob("description/*.xacro")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world.in")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models", "dnn"), glob("models/dnn/*")),
        (
            os.path.join("share", package_name, "worlds", "models", "person_photo_model"),
            glob("worlds/models/person_photo_model/*.config")
            + glob("worlds/models/person_photo_model/*.sdf"),
        ),
        (
            os.path.join(
                "share", package_name, "worlds", "models", "person_photo_model",
                "materials", "scripts",
            ),
            glob("worlds/models/person_photo_model/materials/scripts/*"),
        ),
        (
            os.path.join(
                "share", package_name, "worlds", "models", "person_photo_model",
                "materials", "textures",
            ),
            glob("worlds/models/person_photo_model/materials/textures/*"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zach",
    maintainer_email="you@example.com",
    description=(
        "Vision-guided rover: OpenCV perception node + proportional motor "
        "control node over ROS2 topics, for Gazebo sim or Raspberry Pi hardware."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perception_node = vision_bot.perception_node:main",
            "motor_control_node = vision_bot.motor_control_node:main",
            "gpio_motor_driver = vision_bot.gpio_motor_driver:main",
        ],
    },
)
