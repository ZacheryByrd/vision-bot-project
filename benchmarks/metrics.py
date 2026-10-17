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


@dataclass(frozen=True)
class GoalResult:
    goal_id: str
    success: bool
    time_s: float
    path_length_m: float
    final_error_m: float
    recoveries: int


@dataclass(frozen=True)
class GoalSummary:
    n: int
    success_rate: float
    mean_time_s: float
    mean_final_error_m: float
    total_recoveries: int


def summarize_goals(results):
    """Success rate over every goal; time and error means over successes only.

    A failed goal's time is its timeout and its error is wherever it gave
    up, so averaging them in would describe the failure, not the navigation.
    Recoveries are counted over every goal, since that is where they happen.
    """
    if not results:
        return GoalSummary(n=0, success_rate=0.0, mean_time_s=0.0,
                           mean_final_error_m=0.0, total_recoveries=0)
    ok = [r for r in results if r.success]
    return GoalSummary(
        n=len(results),
        success_rate=len(ok) / len(results),
        mean_time_s=sum(r.time_s for r in ok) / len(ok) if ok else 0.0,
        mean_final_error_m=sum(r.final_error_m for r in ok) / len(ok) if ok else 0.0,
        total_recoveries=sum(r.recoveries for r in results),
    )
