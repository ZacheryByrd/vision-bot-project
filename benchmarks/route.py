"""Scripted routes and the steering law that drives them (pure, no ROS).

The driver steers from Gazebo's ground-truth pose, the way a person at the
keyboard steers by watching the robot. Ground truth only picks the wheel
commands; it never reaches odometry, the EKF, SLAM, or Nav2, so it cannot
flatter the estimates being measured. Using it makes every run drive the
same physical route.

Waypoints are (x, y) in the nav_world world frame; the robot spawns at the
origin facing +x. Both routes keep at least 0.4 m from walls and obstacles
(see the layout comment in worlds/nav_world.world).
"""

import math

from metrics import Pose2D

MAX_LIN = 0.25         # m/s
MAX_ANG = 1.0          # rad/s
K_ANG = 2.0            # rad/s per rad of heading error
K_LIN = 0.8            # m/s per m of remaining distance (slows into waypoints)
TURN_IN_PLACE = 0.4    # rad: above this heading error, rotate on the spot
REACHED_TOL = 0.12     # m

# A rectangle in the open middle of the world, driven twice (22.8 m): long
# straights plus eight 90-degree turns, where wheel slip shows up.
_LOOP = [(1.0, 0.0), (1.0, -2.2), (-2.0, -2.2), (-2.0, 0.5), (0.0, 0.5), (0.0, 0.0)]

# A tour through every room for mapping: east doorway, SE room, east area,
# NE corridor, north-centre, NW room, west side, SW alcove, south-centre.
_EXPLORE = [
    (0.8, -0.3), (2.1, -0.3),                              # east doorway
    (2.1, -2.0), (3.0, -2.4), (4.45, -2.4), (4.45, -4.4),  # SE room, round box_b
    (3.0, -4.4), (2.1, -2.0), (2.1, -0.3),                 # and back to the doorway
    (3.8, -0.2), (4.2, 2.4), (4.2, 4.2),                   # east area
    (2.5, 4.2), (1.0, 4.3), (0.9, 2.6), (-0.4, 1.7),       # NE corridor, north-centre
    (-2.0, 1.7), (-2.6, 2.6), (-2.6, 4.3), (-4.3, 4.3),    # NW room
    (-4.3, 1.6), (-2.0, 1.6), (-0.4, 1.6),                 # out of the NW room
    (-0.4, -0.5), (-3.0, -0.5), (-3.2, -2.0), (-3.8, -3.6),  # west side, SW alcove
    (-3.2, -2.0), (-1.6, -2.0), (0.2, -2.2), (0.6, -4.2),  # south-centre
    (0.5, -1.0), (0.0, 0.0),                               # back to the start
]

ROUTES = {
    "drift_loop": _LOOP * 2,
    "explore": _EXPLORE,
}


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def steer(pose, goal):
    """(linear, angular) velocity that drives pose toward the goal point."""
    dx, dy = goal[0] - pose.x, goal[1] - pose.y
    error = _wrap(math.atan2(dy, dx) - pose.yaw)
    ang = max(-MAX_ANG, min(MAX_ANG, K_ANG * error))
    if abs(error) > TURN_IN_PLACE:
        return 0.0, ang
    lin = min(MAX_LIN, K_LIN * math.hypot(dx, dy)) * math.cos(error)
    return lin, ang


def reached(pose, goal, tol=REACHED_TOL):
    return math.hypot(goal[0] - pose.x, goal[1] - pose.y) <= tol


def route_length(waypoints, start):
    """Straight-line length of a route from start through every waypoint."""
    points = [start] + list(waypoints)
    return sum(math.dist(a, b) for a, b in zip(points, points[1:]))


def pose_from_odometry(msg):
    """Pose2D from a nav_msgs/Odometry message (planar yaw from the quaternion)."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return Pose2D(p.x, p.y, yaw)
