"""Tests for the pure map parsing in scripts/map_check.py (no ROS needed)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from map_check import occupied_points, parse_pgm  # noqa: E402

# 3 x 2 binary PGM with a comment line, rows top to bottom.
PGM = b"P5\n# CREATOR: map_saver\n3 2\n255\n" + bytes([0, 254, 205, 254, 0, 254])


def test_parse_pgm_reads_header_and_pixels():
    width, height, pixels = parse_pgm(PGM)
    assert (width, height) == (3, 2)
    assert pixels == [0, 254, 205, 254, 0, 254]


def test_parse_pgm_rejects_ascii_pgm():
    with pytest.raises(ValueError):
        parse_pgm(b"P2\n1 1\n255\n0\n")


def test_occupied_points_are_cell_centres_in_world_frame():
    # resolution 1 m, map origin (lower-left corner) at (-1, -1).
    width, height, pixels = parse_pgm(PGM)
    points = occupied_points(pixels, width, height, resolution=1.0, origin=(-1.0, -1.0),
                             occupied_thresh=0.65, negate=False)
    # Row 0 is the top of the image, i.e. the highest y.
    assert sorted(points) == sorted([(-0.5, 0.5), (0.5, -0.5)])


def test_unknown_and_free_cells_are_not_occupied():
    points = occupied_points([205, 254], 2, 1, resolution=0.05, origin=(0.0, 0.0),
                             occupied_thresh=0.65, negate=False)
    assert points == []


def test_negate_flips_the_colour_meaning():
    points = occupied_points([255, 0], 2, 1, resolution=1.0, origin=(0.0, 0.0),
                             occupied_thresh=0.65, negate=True)
    assert points == [(0.5, 0.5)]
