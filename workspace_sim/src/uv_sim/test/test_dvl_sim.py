"""``dvl_sim``: the validity rule and the beam synthesis (AQUA-SLAM's adapter).

Two claims are worth pinning because both have a plausible wrong version that
looks right:

* **validity comes from altitude.**  The obvious ``velocity_covariance[0] >= 0``
  is always true in this engine (``ROS2Interface.cpp:211-214`` writes
  ``sigma^2 > 0`` unconditionally), so it would let water-layer samples through as
  if they were bottom-tracked.
* **the beam elevation is ``alpha = 60 deg``**, not the scene's 30 deg value:
  feeding the off-vertical angle where AQUA-SLAM expects the elevation scales
  horizontal velocity by ``cos(30)/cos(60) = 1.73``.  The round trip through
  :func:`uv_sim.geometry.beam_velocity_to_body` is the check that the synthesized
  beams invert back to the velocity they were built from.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import rclpy

from stonefish_ros2.msg import DVL as SimDvl

from uv_sim.dvl_sim import (
    DVL_ALPHA_RAD,
    DVL_BEAM_ANGLE_DEG,
    DVL_BEAM_AZIMUTHS_DEG,
    DVL_BEAM_MATRIX,
    DvlSimNode,
    beam_velocities,
    velocity_valid_from_altitude,
)
from uv_sim.geometry import beam_velocity_to_body


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class _Recorder:
    def __init__(self) -> None:
        self.messages: list = []

    def publish(self, msg) -> None:
        self.messages.append(msg)


def _sim_dvl(velocity=(0.3, -0.2, 0.05), altitude=12.5, covariance=2.25e-6) -> SimDvl:
    msg = SimDvl()
    msg.header.frame_id = "dvl"
    msg.header.stamp.sec = 7
    msg.velocity.x, msg.velocity.y, msg.velocity.z = velocity
    msg.altitude = altitude
    msg.velocity_covariance[0] = covariance
    return msg


# --------------------------------------------------------------------------- #
# Validity
# --------------------------------------------------------------------------- #
def test_validity_follows_the_altitude_not_the_covariance() -> None:
    assert velocity_valid_from_altitude(12.5) is True
    assert velocity_valid_from_altitude(0.0) is True
    assert velocity_valid_from_altitude(-1.0) is False   # engine's "no bottom lock"


def test_no_bottom_lock_is_reported_even_though_covariance_is_positive(ros_context) -> None:
    node = DvlSimNode(node_name="dvl_sim_invalid")
    try:
        node.publisher = _Recorder()
        node._dvl_cb(_sim_dvl(altitude=-1.0))

        out = node.publisher.messages[-1]
        assert out.velocity_valid is False
        assert node.invalid_count == 1
    finally:
        node.destroy_node()


# --------------------------------------------------------------------------- #
# Beam geometry
# --------------------------------------------------------------------------- #
def test_alpha_is_the_elevation_not_the_scene_angle() -> None:
    assert DVL_BEAM_ANGLE_DEG == pytest.approx(30.0)
    assert math.degrees(DVL_ALPHA_RAD) == pytest.approx(60.0)
    assert DVL_BEAM_AZIMUTHS_DEG == (45.0, 135.0, 225.0, 315.0)


def test_beams_sit_thirty_degrees_off_the_sensor_z_axis() -> None:
    for row in DVL_BEAM_MATRIX:
        assert float(row[2]) == pytest.approx(math.sin(math.radians(60.0)))
        assert float(np.linalg.norm(row[:2])) == pytest.approx(math.cos(math.radians(60.0)))


@pytest.mark.parametrize("velocity", [
    (0.3, -0.2, 0.05),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
    (-0.4, 0.35, -0.1),
])
def test_synthesized_beams_invert_back_to_the_velocity(velocity) -> None:
    """AQUA-SLAM inverts E with a least-squares solve; it must return the input."""
    beams = beam_velocities(velocity)
    assert len(beams) == 4
    recovered = beam_velocity_to_body(DVL_BEAM_MATRIX, beams)
    assert list(recovered) == pytest.approx(list(velocity), abs=1e-12)


def test_pure_surge_beam_pattern_matches_the_azimuths() -> None:
    """Beam 0/2 are the forward pair at +/-45 deg, beam 1/3 the after pair."""
    beams = beam_velocities((1.0, 0.0, 0.0))
    cos_alpha = math.cos(math.radians(60.0))
    # E[0] = (-cos45*cos_a, sin45*cos_a, sin_a) -> u contributes -cos45*cos_a
    assert beams[0] == pytest.approx(-math.cos(math.radians(45.0)) * cos_alpha)
    assert beams[1] == pytest.approx(-math.cos(math.radians(135.0)) * cos_alpha)
    assert beams[2] == pytest.approx(-math.cos(math.radians(225.0)) * cos_alpha)
    assert beams[3] == pytest.approx(-math.cos(math.radians(315.0)) * cos_alpha)


# --------------------------------------------------------------------------- #
# The message mapping
# --------------------------------------------------------------------------- #
def test_message_carries_the_dvl_frame_velocity_and_altitude(ros_context) -> None:
    node = DvlSimNode(node_name="dvl_sim_mapping")
    try:
        node.publisher = _Recorder()
        node._dvl_cb(_sim_dvl(velocity=(0.3, -0.2, 0.05), altitude=12.5))

        out = node.publisher.messages[-1]
        assert (out.velocity.x, out.velocity.y, out.velocity.z) == pytest.approx(
            (0.3, -0.2, 0.05)
        )
        assert out.altitude == pytest.approx(12.5)
        assert out.velocity_valid is True
        # header is forwarded, not re-stamped
        assert out.header.frame_id == "dvl"
        assert out.header.stamp.sec == 7
        assert list(out.beam_velocity) == pytest.approx(
            beam_velocities((0.3, -0.2, 0.05)), abs=1e-6
        )
        assert node.count == 1
    finally:
        node.destroy_node()


def test_the_published_velocity_is_the_dvl_value_not_the_origin_value(ros_context) -> None:
    """The node must not "fix" the lever arm here: B19 is fixed by *separation*.

    ``/sim/sensor/dvl`` carries the DVL-frame value (which is what the sensor
    measures); ``/zit6/state/vel`` carries the origin value.  Two definitions, two
    topics -- so this relay has to stay a passthrough.
    """
    node = DvlSimNode(node_name="dvl_sim_passthrough")
    try:
        node.publisher = _Recorder()
        node._dvl_cb(_sim_dvl(velocity=(0.5, 0.037, 0.0)))

        out = node.publisher.messages[-1]
        assert out.velocity.y == pytest.approx(0.037)
    finally:
        node.destroy_node()
