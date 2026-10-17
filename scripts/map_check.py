#!/usr/bin/env python3
"""Check a saved map against the world it was built in (Task 2 gate).

Reads a map_server map (YAML + binary PGM), takes the centre of every
occupied cell in world coordinates, and measures its distance to the
nearest wall or obstacle in the Gazebo world file. A phantom wall, a
smeared wall, or a map that drifted out of registration all show up as
occupied cells far from every modeled surface. Reports the fraction of
occupied cells within --tol of a surface and writes an overlay PNG.

The map frame equals the world frame because mapping starts with the robot
at the world origin facing +x (nav_sim.launch.py's default spawn pose).

    python3 scripts/map_check.py \
        --map ros2_ws/src/vision_bot_nav/maps/nav_world.yaml \
        --world ros2_ws/src/vision_bot_nav/worlds/nav_world.world --out map_check.png
"""

import argparse
import math
from pathlib import Path

from scan_overlay import Cylinder, distance_to_surface, load_shapes


def parse_pgm(data):
    """(width, height, pixels) of an 8-bit binary (P5) PGM; pixels row-major, top row first."""
    if not data.startswith(b"P5"):
        raise ValueError("not a binary (P5) PGM")
    fields, i = [], 2
    while len(fields) < 3:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            i = data.index(b"\n", i) + 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        fields.append(int(data[i:j]))
        i = j
    width, height, maxval = fields
    if maxval > 255:
        raise ValueError("16-bit PGM not supported")
    i += 1  # the single whitespace byte after maxval
    return width, height, list(data[i:i + width * height])


def occupied_points(pixels, width, height, resolution, origin, occupied_thresh, negate):
    """World (x, y) centres of occupied cells, using map_server's trinary rule."""
    points = []
    for index, value in enumerate(pixels):
        p = value / 255.0 if negate else (255 - value) / 255.0
        if p > occupied_thresh:
            row, col = divmod(index, width)
            points.append((origin[0] + (col + 0.5) * resolution,
                           origin[1] + (height - row - 0.5) * resolution))
    return points


def _plot(pixels, width, height, meta, shapes, points, near, out_path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Circle, Polygon

    res, (ox, oy) = meta["resolution"], meta["origin"][:2]
    img = np.array(pixels, dtype=np.uint8).reshape(height, width)
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(img, cmap="gray", vmin=0, vmax=255, origin="upper",
              extent=(ox, ox + width * res, oy, oy + height * res))
    for s in shapes:
        if isinstance(s, Cylinder):
            ax.add_patch(Circle((s.cx, s.cy), s.r, fill=False, color="tab:cyan", lw=1.0))
            continue
        c, si = math.cos(s.yaw), math.sin(s.yaw)
        corners = [(u * s.sx / 2, v * s.sy / 2) for u, v in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        ax.add_patch(Polygon([(s.cx + c * u - si * v, s.cy + si * u + c * v) for u, v in corners],
                             closed=True, fill=False, color="tab:cyan", lw=1.0))
    far = [p for p, ok in zip(points, near) if not ok]
    if far:
        ax.scatter(*zip(*far), s=8, color="tab:red", label="occupied cell off every surface")
        ax.legend(loc="upper right", fontsize=8)
    ax.set_xlim(-5.6, 5.6)
    ax.set_ylim(-5.6, 5.6)
    ax.set_xlabel("world x (m)")
    ax.set_ylabel("world y (m)")
    ax.set_title(title)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)


def main():
    import yaml

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--map", required=True, help="map_server YAML file")
    parser.add_argument("--world", required=True, help="Gazebo world file")
    parser.add_argument("--tol", type=float, default=0.10, help="metres")
    parser.add_argument("--out", default="map_check.png")
    args = parser.parse_args()

    meta = yaml.safe_load(Path(args.map).read_text())
    image = Path(args.map).parent / meta["image"]
    width, height, pixels = parse_pgm(image.read_bytes())
    points = occupied_points(pixels, width, height, meta["resolution"], meta["origin"][:2],
                             meta["occupied_thresh"], bool(meta.get("negate", 0)))
    shapes = load_shapes(Path(args.world).read_text())
    nearest = sorted(min(distance_to_surface(x, y, s) for s in shapes) for x, y in points)
    near = [min(distance_to_surface(x, y, s) for s in shapes) <= args.tol for x, y in points]
    frac = sum(near) / len(points) if points else 0.0

    print(f"map: {width} x {height} cells at {meta['resolution']} m, origin {meta['origin'][:2]}")
    print(f"occupied cells: {len(points)}")
    print(f"within {args.tol} m of a modeled surface: {sum(near)}/{len(points)} = {frac:.3f}")
    if nearest:
        print(f"distance to nearest surface: median {nearest[len(nearest) // 2]:.3f} m, "
              f"95th percentile {nearest[int(0.95 * (len(nearest) - 1))]:.3f} m, "
              f"max {nearest[-1]:.3f} m")
    _plot(pixels, width, height, meta, shapes, points, near, args.out,
          f"{image.name} over {Path(args.world).name}: {frac:.1%} of {len(points)} "
          f"occupied cells within {args.tol} m")
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
