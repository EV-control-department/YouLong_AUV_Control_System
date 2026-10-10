"""The line laser: the scene mount, the beam convention and the point cloud.

The laser is a Stonefish ``multibeam`` -- one plane of rays, i.e. one line on the
seabed -- and ``laser_sim`` turns its ``LaserScan`` into stripe points.  Four
claims are pinned here, each because a plausible wrong version exists:

* **the mount really is "straight down, line across the track"**: the sensor
  ``x`` axis (the fan's centre ray) must map to body ``+z`` and the sensor ``y``
  axis (the direction the line runs in) to body ``+y``.  A pure yaw or a
  ``rpy`` with the wrong sign gives a fan that is off by 90 degrees and still
  looks plausible in a plot;
* **angles come from the endpoints**: ``angle_min``/``angle_max`` are inclusive,
  so the spacing is ``(max - min) / (N - 1)`` (``ROS2Interface.cpp``);
* **a miss is dropped, not published at range_max**: the engine marks it with
  ``intensity = 0`` and ``range = inf``; keeping it would draw a fake ring of
  "bottom" at exactly 15 m and quietly poison any map built from it;
* **the points lie in the laser plane** (``z = 0``, sensor frame), which is what
  makes the cloud a structured-light profile rather than a body-frame point set
  (see the module docstring of ``uv_sim.laser_sim`` for why it is the sensor
  frame).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import LaserScan

from uv_sim.geometry import euler_rpy_to_matrix, parse_scn_sensors
from uv_sim.laser_sim import (
    LaserSimNode,
    apply_attenuation,
    detection_probability,
    scan_angles,
    scan_to_points,
    valid_mask,
)

# The two topic names, pinned here because the node inlines them as literals
# (the docs contract checker resolves literals only -- see
# scripts/check_docs_contract.py and the note in uv_sim/laser_sim.py).
SCAN_TOPIC = '/auv/laser_scan'
POINTS_TOPIC = '/sim/sensor/laser_points'

SCENE_RELATIVE_PATH = Path("workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn")

#: The mount, in the body frame: on the centreline, forward of the down stereo
#: (x=-0.13) and just below the deepest centre-line hull surface (z~0.18).
LASER_XYZ = (0.05, 0.0, 0.19)
LASER_RPY = (0.0, -math.pi / 2, 0.0)
LASER_RATE_HZ = 30.0
LASER_FOV_DEG = 87.19


def _repository_root() -> Path:
    for candidate in Path(__file__).resolve().parents:
        if (candidate / ".git").exists():
            return candidate
    return Path(__file__).resolve().parents[0]


@pytest.fixture(scope="module")
def scene() -> ET.Element:
    path = _repository_root() / SCENE_RELATIVE_PATH
    if not path.exists():
        pytest.skip(f"vehicle scene not available in this checkout: {path}")
    return ET.parse(path).getroot()


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


def _sensor(scene: ET.Element, name: str) -> ET.Element:
    for element in scene.iter("sensor"):
        if element.get("name") == name:
            return element
    raise AssertionError(f"sensor {name!r} not found in the vehicle scene")


def _scan(ranges, angle_min=-0.1, angle_max=0.1, intensities=None) -> LaserScan:
    msg = LaserScan()
    msg.header.frame_id = "line_laser"
    msg.header.stamp.sec = 3
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    count = len(ranges)
    msg.angle_increment = (
        0.0 if count < 2 else (angle_max - angle_min) / (count - 1)
    )
    msg.range_min = 0.3
    msg.range_max = 15.0
    msg.ranges = [float(value) for value in ranges]
    msg.intensities = (
        [1.0] * count if intensities is None else [float(v) for v in intensities]
    )
    return msg


# --------------------------------------------------------------------------- #
# The scene: "straight down, one line across the track"
# --------------------------------------------------------------------------- #
def test_laser_is_a_single_plane_multibeam_looking_straight_down(scene) -> None:
    laser = _sensor(scene, "line_laser")
    assert laser.get("type") == "multibeam", (
        "the line laser is expressed as Stonefish's single-plane raycast sensor; "
        "a camera-based approximation cannot measure the bottom"
    )
    assert float(laser.get("rate", "nan")) == pytest.approx(LASER_RATE_HZ)

    origin = laser.find("origin")
    assert origin is not None, "line_laser has no <origin>"
    xyz = tuple(float(value) for value in origin.get("xyz", "").split())
    rpy = tuple(float(value) for value in origin.get("rpy", "").split())
    assert xyz == pytest.approx(LASER_XYZ, abs=1e-6)
    # 1e-4: the scene writes pi/2 as "1.5708" (like the rest of the repo's
    # mounts), so the stored angle is off the true pi/2 by 3.7e-6 rad.
    assert rpy == pytest.approx(LASER_RPY, abs=1e-4), (
        "a pure -90 deg pitch is what makes the fan plane the body y-z plane; "
        "any other mount rotates the line off the across-track direction"
    )

    specs = laser.find("specs")
    assert specs is not None, "line_laser has no <specs>"
    assert float(specs.get("fov", "nan")) == pytest.approx(LASER_FOV_DEG)
    assert int(specs.get("steps", "0")) == 959, (
        "steps is the number of intervals: the engine casts steps+1 rays"
        " (Multibeam.cpp:45)"
    )


def test_sensor_axes_are_down_and_starboard(scene) -> None:
    """``x`` -> body +z (centre ray down), ``y`` -> body +y (the line's direction)."""
    sensor = parse_scn_sensors(_repository_root() / SCENE_RELATIVE_PATH)["line_laser"]
    rotation = euler_rpy_to_matrix(*sensor.rpy)[:3, :3]

    assert rotation[:, 0] == pytest.approx([0.0, 0.0, 1.0], abs=1e-5), (
        "the fan's centre ray must point straight down (body +z, NED)"
    )
    assert rotation[:, 1] == pytest.approx([0.0, 1.0, 0.0], abs=1e-5), (
        "the line must run along body +y, i.e. perpendicular to the heading"
    )
    assert rotation[:, 2] == pytest.approx([-1.0, 0.0, 0.0], abs=1e-5)
    assert float(np.linalg.det(rotation)) == pytest.approx(1.0)


def test_laser_is_below_the_hull_and_above_the_side_plates(scene) -> None:
    """The mount height is a clearance argument, so pin the two surfaces.

    The deepest hull surfaces on the centreline measure ``z ~ 0.15-0.18``
    (``InsBasePlate``, verified by raycasting the scene's own collision meshes);
    the side plates hang to ``z ~ 0.21`` at ``y = +-0.18``.  At
    ``fov/2 = 43.6 deg`` a ray from ``z = 0.19`` reaches ``y = +-0.18`` only at
    ``z ~ 0.37``, i.e. well below the plates -- that is why the fan clears the
    hull, and it is the reason the mount is not simply "wherever the camera is".
    """
    origin = _sensor(scene, "line_laser").find("origin")
    assert origin is not None
    mount_z = float(origin.get("xyz", "").split()[2])

    assert mount_z >= 0.18, "the fan would start inside the hull"
    half_fov = math.radians(LASER_FOV_DEG / 2)
    # z where the outermost ray passes the side plates' y
    z_at_side_plate = mount_z + 0.18 / math.tan(half_fov)
    assert z_at_side_plate > 0.21, (
        "the outermost ray would cross the side plate (z=0.21) instead of "
        "passing under it"
    )


def test_the_range_floor_keeps_the_hull_out_of_the_scan(scene) -> None:
    """``distance_min = 0.3`` is the second line of defence against self-hits."""
    ranges = _sensor(scene, "line_laser").find("range")
    assert ranges is not None, "line_laser has no <range>"
    assert float(ranges.get("distance_min", "0")) == pytest.approx(0.3)
    assert float(ranges.get("distance_max", "0")) == pytest.approx(15.0)


def test_scan_topic_is_the_one_the_node_subscribes_to(scene) -> None:
    publisher = _sensor(scene, "line_laser").find("ros_publisher")
    assert publisher is not None
    assert publisher.get("topic") == SCAN_TOPIC


# --------------------------------------------------------------------------- #
# Polar -> Cartesian
# --------------------------------------------------------------------------- #
def test_angles_span_both_endpoints_inclusively() -> None:
    angles = scan_angles(_scan([1.0] * 5, angle_min=-0.4, angle_max=0.4))
    assert list(angles) == pytest.approx([-0.4, -0.2, 0.0, 0.2, 0.4])


def test_a_single_beam_scan_does_not_divide_by_zero() -> None:
    assert list(scan_angles(_scan([1.0]))) == pytest.approx([-0.1])
    assert scan_angles(_scan([])).shape == (0,)


def test_points_lie_in_the_sensor_plane_on_their_own_rays() -> None:
    scan = _scan([1.0, 2.0, 3.0], angle_min=0.0, angle_max=math.pi / 2)
    points = scan_to_points(scan)

    assert points.shape == (3, 3)
    # z is identically zero: this is a profile in the laser plane
    assert points[:, 2] == pytest.approx([0.0, 0.0, 0.0])
    # x = cos(theta) * r, y = sin(theta) * r (the engine's own PCL convention)
    assert points[0] == pytest.approx([1.0, 0.0, 0.0])
    assert points[2] == pytest.approx([0.0, 3.0, 0.0])
    # every point keeps its own range
    assert np.linalg.norm(points, axis=1) == pytest.approx([1.0, 2.0, 3.0])


def test_a_miss_is_dropped_not_placed_at_range_max() -> None:
    """``inf`` + ``intensity = 0`` is the engine saying "no bottom in range"."""
    scan = _scan([1.0, float("inf"), 3.0], intensities=[1.0, 0.0, 1.0])
    assert list(valid_mask(scan)) == [True, False, True]
    assert scan_to_points(scan).shape == (2, 3)


def test_infinite_range_alone_is_enough_to_drop_a_beam() -> None:
    """Even if a message carried no intensities, ``inf`` must not become a point."""
    scan = _scan([1.0, float("inf")])
    scan.intensities = []
    assert list(valid_mask(scan)) == [True, False]


def test_intensity_zero_alone_is_enough_to_drop_a_beam() -> None:
    """And the engine's own marker works without relying on the range value."""
    scan = _scan([1.0, 15.0], intensities=[1.0, 0.0])
    assert list(valid_mask(scan)) == [True, False]


def test_an_all_miss_scan_yields_an_empty_cloud() -> None:
    scan = _scan([float("inf")] * 4, intensities=[0.0] * 4)
    points = scan_to_points(scan)
    assert points.shape == (0, 3)


# --------------------------------------------------------------------------- #
# Water attenuation (Beer-Lambert round-trip loss -> distance-dependent detection)
# --------------------------------------------------------------------------- #
def test_attenuation_kills_far_points_more_than_near() -> None:
    """The water signature: near is always detected, far thins out and vanishes."""
    z = np.array([0.5, 1.0, 3.0, 5.0, 10.0])
    prob = detection_probability(z, attenuation_c=0.2, ref_snr=10.0,
                                 threshold=3.0, spread=1.0, ref_distance=1.0)
    assert prob[0] == pytest.approx(1.0, abs=1e-3), "near returns survive"
    assert prob[2] > prob[3] > prob[4], "detection must fall with distance"
    assert prob[4] < 0.05, "far returns are effectively lost"


def test_murky_water_shortens_the_effective_range() -> None:
    """A larger attenuation coefficient c reaches the limit sooner."""
    clear = detection_probability(np.array([3.0]), 0.05)[0]
    murky = detection_probability(np.array([3.0]), 0.5)[0]
    assert clear > murky, "clearer water keeps more of the stripe at 3 m"
    assert clear == pytest.approx(1.0, abs=1e-3)
    assert murky < 0.5


def test_attenuation_disabled_is_a_pure_geometric_adapter() -> None:
    """c <= 0 must leave the geometric validity untouched (no random drops)."""
    rng = np.random.default_rng(1)
    out = apply_attenuation(np.ones(500, dtype=bool), np.full(500, 15.0), 0.0, rng)
    assert out.all(), "attenuation_c=0 must not drop any point"


def test_attenuation_is_stochastic_not_a_hard_cutoff() -> None:
    """Near the limit the stripe gets sparser, not sharply truncated."""
    rng = np.random.default_rng(42)
    # 4 m in coastal water: a mid-range, partially-surviving distance.
    out = apply_attenuation(np.ones(2000, dtype=bool), np.full(2000, 4.0), 0.3, rng)
    survived = float(out.mean())
    assert 0.0 < survived < 1.0, (
        f"at the knee of the curve some points must survive and some must not "
        f"(got {survived:.2f}); a hard cutoff would give 0 or 1"
    )


# --------------------------------------------------------------------------- #
# The node
# --------------------------------------------------------------------------- #
def test_node_publishes_a_cloud_in_the_sensor_frame(ros_context) -> None:
    node = LaserSimNode(node_name="laser_sim_mapping")
    try:
        node.publisher = _Recorder()
        node._scan_cb(_scan([1.0, 2.0, float("inf")], intensities=[1.0, 1.0, 0.0]))

        cloud = node.publisher.messages[-1]
        assert cloud.header.frame_id == "line_laser", "header must be forwarded"
        assert cloud.header.stamp.sec == 3
        assert cloud.height == 1
        assert cloud.width == 2, "the missed beam must not become a point"
        assert cloud.is_dense is True, "every published point came off the bottom"
        assert node.count == 1
        assert node.missed == 1
    finally:
        node.destroy_node()


def test_node_uses_the_owned_points_topic(ros_context) -> None:
    node = LaserSimNode(node_name="laser_sim_topics")
    try:
        assert node.publisher.topic_name == POINTS_TOPIC
        assert node.get_subscriptions_info_by_topic(SCAN_TOPIC), (
            f"{SCAN_TOPIC} is not subscribed"
        )
    finally:
        node.destroy_node()
