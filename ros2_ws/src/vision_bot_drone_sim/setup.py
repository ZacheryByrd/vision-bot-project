import os
from glob import glob

from setuptools import find_packages, setup

package_name = "vision_bot_drone_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "models", "drone"), glob("models/drone/*")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zach",
    maintainer_email="you@example.com",
    description="Simulated drone flown along seeded random waypoints, with ground truth.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["drone_flight = vision_bot_drone_sim.drone_flight:main"]},
)
