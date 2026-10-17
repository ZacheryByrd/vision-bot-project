#!/usr/bin/env python3
"""Plot one lidar scan over the world's walls and obstacles (Task 1 gate).

RViz cannot draw the Gazebo world, so this script checks "the scan matches
the walls" directly: it reads the box and cylinder collision shapes of the
world file, places each scan return in the world frame using the robot's
known pose (the spawn pose, before the robot has moved), and reports the
fraction of returns that land within --tol metres of a modeled surface.
It also writes a PNG of the overlay.

Assumes every shape spans the scan plane's height (true for nav_world:
all walls and obstacles start at the floor and are 0.4 m or taller).

Run inside the container with the sim up:
    python3 scripts/scan_overlay.py \
        --world ros2_ws/src/vision_bot_nav/worlds/nav_world.world \
        --pose 0 0 0 --out scan_overlay.png
"""

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass


@dataclass(frozen=True)
class Box:
    cx: float
    cy: float
    yaw: float
    sx: float
    sy: float


@dataclass(frozen=True)
class Cylinder:
    cx: float
    cy: float
    r: float


def _pose2d(element):
    """(x, y, yaw) from an element's <pose> child; identity if absent."""
    pose = element.find("pose") if element is not None else None
    if pose is None or not pose.text:
        return (0.0, 0.0, 0.0)
    x, y, _z, _roll, _pitch, yaw = (float(v) for v in pose.text.split())
    return (x, y, yaw)


def _compose(a, b):
    """Pose b expressed in frame a, returned in a's parent frame."""
    ax, ay, at = a
    bx, by, bt = b
    return (
        ax + math.cos(at) * bx - math.sin(at) * by,
        ay + math.sin(at) * bx + math.cos(at) * by,
        at + bt,
    )


def load_shapes(sdf_text):
    """Box and cylinder collision shapes of every model in an SDF world."""
    root = ET.fromstring(sdf_text)
    shapes = []
    for model in root.iter("model"):
        model_pose = _pose2d(model)
        for link in model.findall("link"):
            link_pose = _compose(model_pose, _pose2d(link))
            for collision in link.findall("collision"):
                x, y, yaw = _compose(link_pose, _pose2d(collision))
                geometry = collision.find("geometry")
                box = geometry.find("box")
                cylinder = geometry.find("cylinder")
                if box is not None:
                    sx, sy, _sz = (float(v) for v in box.find("size").text.split())
                    shapes.append(Box(cx=x, cy=y, yaw=yaw, sx=sx, sy=sy))
                elif cylinder is not None:
                    r = float(cylinder.find("radius").text)
                    shapes.append(Cylinder(cx=x, cy=y, r=r))
    return shapes


def distance_to_surface(x, y, shape):
    """Distance from point (x, y) to the outline of a shape, inside or out."""
    dx, dy = x - shape.cx, y - shape.cy
    if isinstance(shape, Cylinder):
        return abs(math.hypot(dx, dy) - shape.r)
    # Rotate into the box frame, then measure against the half extents.
    c, s = math.cos(shape.yaw), math.sin(shape.yaw)
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    qx = abs(lx) - shape.sx / 2
    qy = abs(ly) - shape.sy / 2
    if qx > 0 or qy > 0:
        return math.hypot(max(qx, 0.0), max(qy, 0.0))
    return min(-qx, -qy)


def scan_to_world(ranges, angle_min, angle_increment, range_min, range_max,
                  pose, mount=(0.0, 0.0)):
    """World (x, y) of each valid scan return.

    pose is the robot's (x, y, yaw) in the world; mount is the lidar's
    (x, y) offset in the robot frame (the lidar has no yaw offset).
    """
    sx, sy, yaw = _compose(pose, (mount[0], mount[1], 0.0))
    points = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = yaw + angle_min + i * angle_increment
        points.append((sx + r * math.cos(a), sy + r * math.sin(a)))
    return points


def fraction_within(points, shapes, tol):
    """Fraction of points within tol of at least one shape's outline."""
    if not points:
        return 0.0
    hits = sum(
        1 for x, y in points
        if min(distance_to_surface(x, y, s) for s in shapes) <= tol
    )
    return hits / len(points)


def _read_one_scan(topic, timeout_s):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node("scan_overlay")
    received = []
    node.create_subscription(LaserScan, topic, received.append, qos_profile_sensor_data)
    waited = 0.0
    while not received and waited < timeout_s:
        rclpy.spin_once(node, timeout_sec=0.5)
        waited += 0.5
    node.destroy_node()
    rclpy.shutdown()
    if not received:
        raise SystemExit(f"no LaserScan on {topic} within {timeout_s} s")
    return received[0]


def _plot(shapes, points, shapes_tol_hits, pose, title, out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    fig, ax = plt.subplots(figsize=(8, 8))
    for s in shapes:
        if isinstance(s, Cylinder):
            ax.add_patch(Circle((s.cx, s.cy), s.r, fill=False, color="0.3", lw=1.5))
            continue
        c, si = math.cos(s.yaw), math.sin(s.yaw)
        corners = [(sx_ * s.sx / 2, sy_ * s.sy / 2)
                   for sx_, sy_ in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        ax.add_patch(Polygon(
            [(s.cx + c * u - si * v, s.cy + si * u + c * v) for u, v in corners],
            closed=True, fill=False, color="0.3", lw=1.5))
    hit = [p for p, h in zip(points, shapes_tol_hits) if h]
    miss = [p for p, h in zip(points, shapes_tol_hits) if not h]
    if hit:
        ax.scatter(*zip(*hit), s=6, color="tab:green", label="return on a modeled surface")
    if miss:
        ax.scatter(*zip(*miss), s=10, color="tab:red", label="return off every surface")
    x, y, yaw = pose
    ax.plot(x, y, "o", color="tab:blue", ms=8, label="robot (spawn pose)")
    ax.arrow(x, y, 0.5 * math.cos(yaw), 0.5 * math.sin(yaw),
             width=0.03, color="tab:blue")
    ax.set_aspect("equal")
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, lw=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--world", required=True, help="Gazebo world file")
    parser.add_argument("--pose", nargs=3, type=float, default=[0.0, 0.0, 0.0],
                        metavar=("X", "Y", "YAW"), help="robot pose in the world")
    parser.add_argument("--mount", nargs=2, type=float, default=[0.0, 0.0],
                        metavar=("X", "Y"), help="lidar offset in the robot frame")
    parser.add_argument("--tol", type=float, default=0.05, help="metres")
    parser.add_argument("--topic", default="scan")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--out", default="scan_overlay.png")
    args = parser.parse_args()

    with open(args.world) as f:
        shapes = load_shapes(f.read())
    scan = _read_one_scan(args.topic, args.timeout)
    pose = tuple(args.pose)
    points = scan_to_world(scan.ranges, scan.angle_min, scan.angle_increment,
                           scan.range_min, scan.range_max, pose, tuple(args.mount))
    nearest = [min(distance_to_surface(x, y, s) for s in shapes) for x, y in points]
    hits = [d <= args.tol for d in nearest]
    frac = fraction_within(points, shapes, args.tol)
    median = sorted(nearest)[len(nearest) // 2] if nearest else float("nan")

    print(f"returns: {len(scan.ranges)}  valid: {len(points)}")
    print(f"within {args.tol} m of a modeled surface: {sum(hits)}/{len(points)} = {frac:.3f}")
    print(f"median distance to nearest surface: {median:.4f} m")
    _plot(shapes, points, hits, pose,
          f"/{args.topic.lstrip('/')} over {args.world.split('/')[-1]}: "
          f"{frac:.1%} of {len(points)} returns within {args.tol} m",
          args.out)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
