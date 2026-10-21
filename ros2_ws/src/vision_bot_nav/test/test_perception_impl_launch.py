"""perception_impl picks the perception_node package in every launch file that starts it.

Loads each launch file and resolves the perception node's package for the
default, python and cpp. Needs the built workspace sourced (the launch
files look up package share directories).

Run: python3 -m pytest ros2_ws/src/vision_bot_nav/test
"""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

SRC = Path(__file__).resolve().parents[2]
LAUNCH_FILES = [
    SRC / "vision_bot" / "launch" / "sim_launch.py",
    SRC / "vision_bot" / "launch" / "hardware_launch.py",
    SRC / "vision_bot_nav" / "launch" / "nav_sim.launch.py",
]


def entities(launch_file):
    spec = importlib.util.spec_from_file_location(launch_file.stem.replace(".", "_"), launch_file)
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
        return module.generate_launch_description().entities
    except Exception as err:  # the package lookups need the sourced workspace
        pytest.skip(f"cannot load {launch_file.name} (workspace not sourced?): {err}")


def walk(items):
    """Every entity, including those inside groups (nav_sim nests its nodes)."""
    for item in items:
        yield item
        actions = getattr(item, "_GroupAction__actions", None)
        if isinstance(actions, list):
            yield from walk(actions)


def resolve(context, value):
    """A Node field: a plain string, one substitution, or a list of them."""
    if isinstance(value, str):
        return value
    if hasattr(value, "perform"):
        return value.perform(context)
    return perform_substitutions(context, value)


def perception_package(launch_file, impl):
    items = list(walk(entities(launch_file)))
    declared = [a for a in items if isinstance(a, DeclareLaunchArgument)
                and a.name == "perception_impl"]
    assert len(declared) == 1, f"{launch_file.name} must declare perception_impl once"
    context = LaunchContext()
    context.launch_configurations["perception_impl"] = (
        impl if impl is not None else resolve(context, declared[0].default_value))
    nodes = [n for n in items if isinstance(n, Node)
             and resolve(context, n.node_executable) == "perception_node"]
    assert len(nodes) == 1, f"{launch_file.name} must start one perception_node"
    return resolve(context, nodes[0].node_package)


@pytest.mark.parametrize("launch_file", LAUNCH_FILES, ids=lambda p: p.name)
@pytest.mark.parametrize("impl, package", [(None, "vision_bot"), ("python", "vision_bot"),
                                           ("cpp", "vision_bot_perception")])
def test_perception_impl_selects_package(launch_file, impl, package):
    assert perception_package(launch_file, impl) == package
