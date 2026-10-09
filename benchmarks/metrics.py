"""Pure statistics for the v2 benchmarks. No ROS imports, so CI can test it.

Every number reported in the README or docs comes from a benchmark script
that uses these functions on recorded data (spec invariant 5).
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class ErrorStats:
    mean: float
    max: float
    final: float


def position_errors(estimate, truth):
    """Euclidean xy error between two equally long, index-aligned paths.

    Yaw is ignored. Raises ValueError if the paths differ in length or are
    empty.
    """
    if len(estimate) != len(truth):
        raise ValueError(f"path lengths differ: {len(estimate)} vs {len(truth)}")
    if not estimate:
        raise ValueError("paths are empty")
    errors = [math.hypot(e.x - t.x, e.y - t.y) for e, t in zip(estimate, truth)]
    return ErrorStats(mean=sum(errors) / len(errors), max=max(errors), final=errors[-1])


def relative_to(origin, pose):
    """pose expressed in the frame of origin (both Pose2D in a common frame).

    Odometry starts at the robot's start pose while ground truth is in the
    world frame; expressing each relative to its own first sample puts all
    of them in the same "relative to where the route began" frame.
    """
    dx, dy = pose.x - origin.x, pose.y - origin.y
    c, s = math.cos(origin.yaw), math.sin(origin.yaw)
    yaw = math.atan2(math.sin(pose.yaw - origin.yaw), math.cos(pose.yaw - origin.yaw))
    return Pose2D(c * dx + s * dy, -s * dx + c * dy, yaw)
