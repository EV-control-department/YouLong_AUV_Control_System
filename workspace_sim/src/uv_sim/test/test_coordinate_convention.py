"""Regression tests for the cruise-scene to project-NED conversion."""

import math

import pytest

from uv_sim.coordinate_convention import scene_to_odom_ned, scene_yaw_to_odom_ned


def test_scene_plan_maps_north_up_and_east_right():
    # In the Stonefish cruise scene +Y is physical north and -X is east.
    assert scene_to_odom_ned(0.0, 2.5, 1.2) == (2.5, -0.0, 1.2)
    assert scene_to_odom_ned(-3.0, 0.0, 1.2) == (0.0, 3.0, 1.2)


@pytest.mark.parametrize(
    ("scene_yaw", "expected_ned_yaw"),
    [
        (math.pi / 2.0, 0.0),       # scene +Y = north
        (0.0, -math.pi / 2.0),      # scene +X = west
        (-math.pi / 2.0, -math.pi),  # scene -Y = south
    ],
)
def test_scene_yaw_uses_the_same_quarter_turn_as_position(
        scene_yaw, expected_ned_yaw):
    assert scene_yaw_to_odom_ned(scene_yaw) == pytest.approx(expected_ned_yaw)
