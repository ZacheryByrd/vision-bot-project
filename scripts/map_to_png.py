#!/usr/bin/env python3
"""Convert a map_server map (YAML + PGM) to a PNG and MapMeta JSON for the dashboard.

Browsers cannot show PGM, so the dashboard's map view uses a PNG of the same
pixels plus the numbers it needs to turn a click into a map-frame goal
(dashboard/app/lib/mapTransform.ts). Run once per saved map; standard library
only.

    python3 scripts/map_to_png.py ros2_ws/src/vision_bot_nav/maps/nav_world.yaml \
        dashboard/public/maps/nav_world
"""

import argparse
import json
import re
import struct
import zlib
from pathlib import Path

from map_check import parse_pgm


def _chunk(kind, data):
    return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))


def grey_png(width, height, pixels):
    """An 8-bit greyscale PNG of row-major pixels, top row first."""
    rows = b"".join(b"\x00" + bytes(pixels[r * width:(r + 1) * width]) for r in range(height))
    return (b"\x89PNG\r\n\x1a\n"
            + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
            + _chunk(b"IDAT", zlib.compress(rows, 9))
            + _chunk(b"IEND", b""))


def map_meta(yaml_text, width, height):
    """MapMeta (see mapTransform.ts) from the map YAML and the image size."""
    resolution = float(re.search(r"^resolution:\s*([-\d.eE+]+)", yaml_text, re.M).group(1))
    origin = re.search(r"^origin:\s*\[([^\]]+)\]", yaml_text, re.M).group(1).split(",")
    return {"resolution": resolution, "origin": [float(origin[0]), float(origin[1])],
            "widthPx": width, "heightPx": height}


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("out_stem", type=Path, help="writes <out_stem>.png and <out_stem>.json")
    args = parser.parse_args()
    yaml_text = args.map_yaml.read_text()
    image = re.search(r"^image:\s*(\S+)", yaml_text, re.M).group(1)
    width, height, pixels = parse_pgm((args.map_yaml.parent / image).read_bytes())
    args.out_stem.parent.mkdir(parents=True, exist_ok=True)
    args.out_stem.with_suffix(".png").write_bytes(grey_png(width, height, pixels))
    meta = map_meta(yaml_text, width, height)
    args.out_stem.with_suffix(".json").write_text(json.dumps(meta, indent=2) + "\n")
    print(f"{width} x {height} px, {meta['resolution']} m/px, origin {meta['origin']} -> "
          f"{args.out_stem}.png, {args.out_stem}.json")


if __name__ == "__main__":
    main()
