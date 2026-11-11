"""The drone's flight: seeded random waypoints and the pose at any time (pure).

The drone flies straight legs between waypoints at a constant speed per leg
and loops back to the first waypoint after the last one. The whole path is a
function of the seed, so a seed replays a flight exactly (spec section 4.1).
"""

import math
import random
from typing import NamedTuple


class Waypoint(NamedTuple):
    x: float
    y: float
    z: float
    speed: float  # m/s on the leg that ends at this waypoint


def make_waypoints(seed, bounds, alt=(2.0, 6.0), speed=(0.5, 2.0), n=50):
    """n waypoints drawn uniformly inside bounds = (x_min, x_max, y_min, y_max)."""
    rng = random.Random(seed)
    x_min, x_max, y_min, y_max = bounds
    return [Waypoint(rng.uniform(x_min, x_max), rng.uniform(y_min, y_max),
                     rng.uniform(*alt), rng.uniform(*speed)) for _ in range(n)]


def _legs(waypoints):
    """(start, end, duration) for every leg, the last one closing the loop."""
    legs = []
    for a, b in zip(waypoints, waypoints[1:] + waypoints[:1]):
        length = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        legs.append((a, b, length / b.speed))
    return legs


def pose_at(waypoints, t):
    """((x, y, z), (vx, vy, vz)) at t seconds after the start of the flight."""
    legs = [leg for leg in _legs(waypoints) if leg[2] > 0.0]  # skip zero-length legs
    t = t % sum(duration for _, _, duration in legs)
    for a, b, duration in legs:
        if t <= duration:
            f = t / duration
            pos = (a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f, a.z + (b.z - a.z) * f)
            vel = ((b.x - a.x) / duration, (b.y - a.y) / duration, (b.z - a.z) / duration)
            return pos, vel
        t -= duration
    a, b, _ = legs[-1]  # only reached through floating-point rounding at the period's end
    return (b.x, b.y, b.z), (0.0, 0.0, 0.0)
