"""Tests for scripts/map_to_png.py (pure, no ROS needed)."""

import struct
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from map_to_png import grey_png, map_meta  # noqa: E402


def chunks(png):
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    i, out = 8, {}
    while i < len(png):
        (length,) = struct.unpack(">I", png[i:i + 4])
        kind, data = png[i + 4:i + 8], png[i + 8:i + 8 + length]
        (crc,) = struct.unpack(">I", png[i + 8 + length:i + 12 + length])
        assert crc == zlib.crc32(kind + data)
        out.setdefault(kind, b"")
        out[kind] += data
        i += 12 + length
    return out


def test_grey_png_holds_the_pixels_row_by_row():
    png = grey_png(3, 2, [0, 254, 205, 254, 0, 254])
    c = chunks(png)
    width, height, depth, colour = struct.unpack(">IIBB", c[b"IHDR"][:10])
    assert (width, height, depth, colour) == (3, 2, 8, 0)
    raw = zlib.decompress(c[b"IDAT"])
    assert raw == bytes([0, 0, 254, 205, 0, 254, 0, 254])      # filter byte 0 per row
    assert b"IEND" in c


def test_map_meta_takes_resolution_and_origin_from_yaml_and_size_from_image():
    yaml_text = "image: nav_world.pgm\nresolution: 0.05\norigin: [-4.97, -5.01, 0]\nnegate: 0\n"
    assert map_meta(yaml_text, 200, 180) == {
        "resolution": 0.05, "origin": [-4.97, -5.01], "widthPx": 200, "heightPx": 180}
