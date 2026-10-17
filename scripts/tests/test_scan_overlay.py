"""Tests for the pure geometry in scripts/scan_overlay.py (no ROS needed)."""

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scan_overlay import (  # noqa: E402
    Box,
    Cylinder,
    distance_to_surface,
    fraction_within,
    load_shapes,
    scan_to_world,
)

WORLD = """<?xml version="1.0"?>
<sdf version="1.6"><world name="w">
  <include><uri>model://ground_plane</uri></include>
  <model name="walls"><static>true</static><pose>1 2 0 0 0 0</pose>
    <link name="link">
      <collision name="a"><pose>0 3 0.25 0 0 0</pose>
        <geometry><box><size>4 0.1 0.5</size></box></geometry></collision>
      <visual name="a_v"><pose>0 3 0.25 0 0 0</pose>
        <geometry><box><size>4 0.1 0.5</size></box></geometry></visual>
    </link>
  </model>
  <model name="rotated"><static>true</static><pose>0 0 0.2 0 0 1.5707963</pose>
    <link name="link"><collision name="c"><pose>1 0 0 0 0 0</pose>
      <geometry><box><size>1 0.5 0.4</size></box></geometry></collision></link>
  </model>
  <model name="cyl"><static>true</static><pose>-2 -1 0.25 0 0 0</pose>
    <link name="link"><collision name="c">
      <geometry><cylinder><radius>0.3</radius><length>0.5</length></cylinder></geometry>
    </collision></link>
  </model>
</world></sdf>"""


def test_load_shapes_reads_collisions_only():
    shapes = load_shapes(WORLD)
    assert len(shapes) == 3


def test_load_shapes_adds_model_pose_to_collision_pose():
    box = load_shapes(WORLD)[0]
    assert isinstance(box, Box)
    assert box.cx == pytest.approx(1.0)
    assert box.cy == pytest.approx(5.0)
    assert (box.sx, box.sy) == pytest.approx((4.0, 0.1))
    assert box.yaw == pytest.approx(0.0)


def test_load_shapes_rotates_collision_offset_by_model_yaw():
    box = load_shapes(WORLD)[1]
    assert box.cx == pytest.approx(0.0, abs=1e-6)
    assert box.cy == pytest.approx(1.0)
    assert box.yaw == pytest.approx(math.pi / 2)


def test_load_shapes_reads_cylinder():
    cyl = load_shapes(WORLD)[2]
    assert cyl == Cylinder(cx=-2.0, cy=-1.0, r=0.3)


def test_distance_zero_on_box_faces():
    box = Box(cx=0.0, cy=0.0, yaw=0.0, sx=2.0, sy=1.0)
    assert distance_to_surface(1.0, 0.0, box) == pytest.approx(0.0)
    assert distance_to_surface(0.0, 0.5, box) == pytest.approx(0.0)


def test_distance_outside_box_face_and_corner():
    box = Box(cx=0.0, cy=0.0, yaw=0.0, sx=2.0, sy=1.0)
    assert distance_to_surface(3.0, 0.0, box) == pytest.approx(2.0)
    assert distance_to_surface(4.0, 4.5, box) == pytest.approx(5.0)


def test_distance_inside_box_is_to_nearest_face():
    box = Box(cx=0.0, cy=0.0, yaw=0.0, sx=2.0, sy=1.0)
    assert distance_to_surface(0.8, 0.0, box) == pytest.approx(0.2)


def test_distance_respects_box_yaw():
    box = Box(cx=0.0, cy=0.0, yaw=math.pi / 2, sx=2.0, sy=1.0)
    assert distance_to_surface(0.0, 1.0, box) == pytest.approx(0.0, abs=1e-9)
    assert distance_to_surface(0.5, 0.0, box) == pytest.approx(0.0, abs=1e-9)
    assert distance_to_surface(1.5, 0.0, box) == pytest.approx(1.0)


def test_distance_to_cylinder():
    cyl = Cylinder(cx=0.0, cy=0.0, r=0.5)
    assert distance_to_surface(1.5, 0.0, cyl) == pytest.approx(1.0)
    assert distance_to_surface(0.0, 0.2, cyl) == pytest.approx(0.3)


def test_scan_to_world_uses_pose_and_skips_invalid_ranges():
    points = scan_to_world(
        ranges=[1.0, math.inf, 0.05, 2.0],
        angle_min=0.0,
        angle_increment=math.pi / 2,
        range_min=0.12,
        range_max=8.0,
        pose=(1.0, 2.0, math.pi / 2),
    )
    assert len(points) == 2
    assert points[0] == pytest.approx((1.0, 3.0))
    assert points[1] == pytest.approx((3.0, 2.0))


def test_scan_to_world_applies_mount_offset():
    points = scan_to_world(
        ranges=[1.0],
        angle_min=0.0,
        angle_increment=0.1,
        range_min=0.12,
        range_max=8.0,
        pose=(0.0, 0.0, math.pi / 2),
        mount=(0.1, 0.0),
    )
    assert points[0] == pytest.approx((0.0, 1.1))


def test_fraction_within_counts_points_near_any_shape():
    shapes = [Box(cx=0.0, cy=0.0, yaw=0.0, sx=2.0, sy=1.0), Cylinder(cx=5.0, cy=0.0, r=0.5)]
    points = [(1.02, 0.0), (5.5, 0.0), (3.0, 3.0), (10.0, 0.0)]
    assert fraction_within(points, shapes, tol=0.05) == pytest.approx(0.5)


def test_fraction_within_empty_scan_is_zero():
    assert fraction_within([], [Cylinder(cx=0.0, cy=0.0, r=1.0)], tol=0.05) == 0.0
