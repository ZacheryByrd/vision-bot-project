import os
from glob import glob

from setuptools import find_packages, setup

package_name = "fleet_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zach",
    maintainer_email="you@example.com",
    description="Bridge between simulated vision_bots (Nav2) and the robot-fleet-management API.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = fleet_bridge.bridge_node:main",
        ],
    },
)
