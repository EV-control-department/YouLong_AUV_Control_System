"""Static assertions on the XUNYUN vehicle scene's sensor contract.

Parses ``stonefish_ros2/Data/xunyun_fixed.scn`` directly (no simulator needed)
and pins the sensor geometry / rates that the sim-real boundary depends on:

* **DVL** — mounted flush with the body frame (``rpy=0``), ``z=+0.1 m`` as on
  the real vehicle, beams emitted along the sensor ``+z`` (``beam_positive_z``),
  30 deg off vertical with Stonefish's fixed 45/135/225/315 azimuths.  This is
  what makes ``/auv/dvl`` carry *body-frame* velocity; the previous mount
  (``rpy=(pi,0,-45deg)`` + default ``beam_positive_z=false``) rotated the
  published velocity 45 deg and flipped y/z, which fed a heading-dependent
  along-track speed into ``basic_motion`` (see ``docs/roadmap.md`` section 1.3).
* **IMU** — 200 Hz with injected noise whose std devs match the AQUA-SLAM
  continuous-time noise densities (``NoiseGyro=1e-3``, ``NoiseAcc=1e-2``).
* **front stereo** — 20 Hz, matching the tank dataset the SLAM settings were
  tuned on; the down stereo stays at 30 Hz.

A silent regression here is invisible in the SIL stack, so keep this test in
the loop.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

SCENE_RELATIVE_PATH = Path(
    "workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn"
)

# Sensor geometry pinned by docs/frames/extrinsics_reference.md (T2).
DVL_XYZ = (-0.375, 0.0, 0.1)
DVL_RPY = (0.0, 0.0, 0.0)
DVL_BEAM_ANGLE_DEG = 30.0
DVL_RATE_HZ = 20.0

IMU_RATE_HZ = 200.0
IMU_ANGULAR_VELOCITY_STD = 0.0141   # 1e-3 rad/s^0.5 * sqrt(200 Hz)
IMU_LINEAR_ACCEL_STD = 0.1414       # 1e-2 m/s^1.5  * sqrt(200 Hz)

FRONT_CAMERA_RATE_HZ = 20.0
DOWN_CAMERA_RATE_HZ = 30.0
ODOMETRY_RATE_HZ = 30.0
PRESSURE_RATE_HZ = 5.0

VECTOR_TOLERANCE = 1e-6
# The scene stores pi as 3.1416, so angles reconstructed from an rpy mount can
# be off by a few 1e-4 deg; 1e-3 deg still catches any real layout change.
ANGLE_TOLERANCE_DEG = 1e-3


def _repository_root() -> Path:
    """Walk up from this file until the repository root (holds ``.git``)."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / ".git").exists():
            return candidate
    return Path(__file__).resolve().parents[0]


def _scene_path() -> Path:
    root = _repository_root()
    return root / SCENE_RELATIVE_PATH


def _vector(text: str) -> tuple[float, float, float]:
    parts = [float(value) for value in text.split()]
    if len(parts) != 3:
        raise AssertionError(f"expected 3 components, got {text!r}")
    return parts[0], parts[1], parts[2]


def _sensor(scene: ET.Element, name: str) -> ET.Element:
    for element in scene.iter("sensor"):
        if element.get("name") == name:
            return element
    raise AssertionError(f"sensor {name!r} not found in the vehicle scene")


@pytest.fixture(scope="module")
def scene() -> ET.Element:
    path = _scene_path()
    if not path.exists():
        pytest.skip(f"vehicle scene not available in this checkout: {path}")
    return ET.parse(path).getroot()


def test_dvl_is_mounted_flush_with_the_body_frame(scene: ET.Element) -> None:
    dvl = _sensor(scene, "dvl")
    origin = dvl.find("origin")
    assert origin is not None, "DVL has no <origin>"

    xyz = _vector(origin.get("xyz", ""))
    rpy = _vector(origin.get("rpy", ""))
    for measured, expected, axis in zip(
        xyz, DVL_XYZ, ("x", "y", "z")
    ):
        assert measured == pytest.approx(expected, abs=VECTOR_TOLERANCE), (
            f"DVL {axis} offset is {measured}, expected {expected}; "
            "/auv/dvl must be body frame"
        )
    for measured, expected, axis in zip(
        rpy, DVL_RPY, ("roll", "pitch", "yaw")
    ):
        assert measured == pytest.approx(expected, abs=VECTOR_TOLERANCE), (
            f"DVL {axis} is {measured}, expected {expected}; a non-zero mount "
            "rotation makes /auv/dvl a rotated frame"
        )


def test_dvl_beams_point_down_through_the_sensor_positive_z(
    scene: ET.Element,
) -> None:
    dvl = _sensor(scene, "dvl")
    specs = dvl.find("specs")
    assert specs is not None, "DVL has no <specs>"

    assert float(specs.get("beam_angle", "nan")) == pytest.approx(
        DVL_BEAM_ANGLE_DEG
    )
    assert specs.get("beam_positive_z", "").lower() in {"true", "1"}, (
        "beam_positive_z must be true: with rpy=0 the sensor +z points down, "
        "and Stonefish's default (false) would emit the beams upwards"
    )
    assert float(dvl.get("rate", "nan")) == pytest.approx(DVL_RATE_HZ)


def test_imu_runs_at_200hz_with_quantified_noise(scene: ET.Element) -> None:
    imu = _sensor(scene, "imu_filter")
    assert float(imu.get("rate", "nan")) == pytest.approx(IMU_RATE_HZ), (
        "AQUA-SLAM wants 200 Hz raw gyro/acc; a low-rate IMU degrades the "
        "preintegration it was tuned for"
    )

    noise = imu.find("noise")
    assert noise is not None, (
        "IMU has no <noise>; a noiseless IMU hides filter problems and makes "
        "sim tuning non-transferable"
    )

    assert float(noise.get("angular_velocity", "nan")) == pytest.approx(
        IMU_ANGULAR_VELOCITY_STD
    )
    assert float(noise.get("linear_acceleration", "nan")) == pytest.approx(
        IMU_LINEAR_ACCEL_STD
    )


@pytest.mark.parametrize(
    "sensor_name, expected_rate",
    (
        ("front_cam_left", FRONT_CAMERA_RATE_HZ),
        ("front_cam_right", FRONT_CAMERA_RATE_HZ),
        ("down_cam_left", DOWN_CAMERA_RATE_HZ),
        ("down_cam_right", DOWN_CAMERA_RATE_HZ),
        ("dynamics", ODOMETRY_RATE_HZ),
        ("pressure", PRESSURE_RATE_HZ),
    ),
)
def test_remaining_sensor_rates_are_unchanged(
    scene: ET.Element, sensor_name: str, expected_rate: float
) -> None:
    sensor = _sensor(scene, sensor_name)
    assert float(sensor.get("rate", "nan")) == pytest.approx(expected_rate)


# ---------------------------------------------------------------------------
# Frame maths reproducing Stonefish's own conventions
# (ScenarioParser.cpp:5189 for <origin rpy>, DVL.cpp:85-89 for the beams).
# ---------------------------------------------------------------------------


def _rotation_from_rpy(rpy: tuple[float, float, float]) -> list[list[float]]:
    """R = Rz(yaw) . Ry(pitch) . Rx(roll), as Stonefish parses <origin rpy>."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matmul(matrix: list[list[float]], vector: tuple) -> tuple:
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _dvl_beams_in_body_frame(scene: ET.Element) -> tuple[list, list]:
    """Return (R_body_from_sensor, beam unit vectors expressed in the body)."""
    dvl = _sensor(scene, "dvl")
    origin = dvl.find("origin")
    specs = dvl.find("specs")
    assert origin is not None and specs is not None

    rotation = _rotation_from_rpy(_vector(origin.get("rpy", "")))
    beam_angle = math.radians(float(specs.get("beam_angle", "nan")))
    direction_factor = (
        1.0 if specs.get("beam_positive_z", "").lower() in {"true", "1"} else -1.0
    )

    beams = []
    for index in range(4):
        azimuth = math.pi / 4.0 + index * math.pi / 2.0
        # DVL.cpp:85-89 -- beam direction in the sensor frame.
        in_sensor_frame = (
            math.cos(azimuth) * math.sin(beam_angle),
            math.sin(azimuth) * math.sin(beam_angle),
            math.cos(beam_angle),
        )
        scaled = tuple(
            direction_factor * component for component in in_sensor_frame
        )
        beams.append(_matmul(rotation, scaled))
    return rotation, beams


def test_dvl_publishes_body_frame_velocity(scene: ET.Element) -> None:
    """Stonefish publishes ``R_body_from_sensor^T . v_body``.

    With the DVL mounted flush with the body frame that reduces to the body
    velocity itself -- which is what ``basic_motion`` consumes through
    ``/zit6/state/vel``.  The pre-T1 mount (``rpy=(pi,0,-45deg)``) made this
    matrix a non-identity rotation, so keep this regression pinned.
    """
    rotation, _ = _dvl_beams_in_body_frame(scene)
    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    for row in range(3):
        for column in range(3):
            assert rotation[row][column] == pytest.approx(
                identity[row][column], abs=VECTOR_TOLERANCE
            ), (
                "DVL sensor frame is rotated against the body frame; "
                "/auv/dvl would then publish a rotated velocity"
            )


def test_dvl_beams_look_down_at_30deg_on_45deg_azimuths(
    scene: ET.Element,
) -> None:
    """Beams must match the real vehicle: down, 30 deg off vertical, 45 deg arcs."""
    _, beams = _dvl_beams_in_body_frame(scene)

    observed_azimuths = []
    for beam in beams:
        norm = math.sqrt(sum(component * component for component in beam))
        unit = tuple(component / norm for component in beam)

        assert unit[2] > 0.0, (
            "beam points up (NED z is down); check beam_positive_z and the rpy"
        )
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, unit[2]))))
        assert tilt == pytest.approx(
            DVL_BEAM_ANGLE_DEG, abs=ANGLE_TOLERANCE_DEG
        )
        observed_azimuths.append(math.degrees(math.atan2(unit[1], unit[0])) % 360.0)

    observed_azimuths.sort()
    for observed, expected in zip(observed_azimuths, (45.0, 135.0, 225.0, 315.0)):
        assert observed == pytest.approx(expected, abs=ANGLE_TOLERANCE_DEG), (
            "beam azimuths in the body frame must be 45/135/225/315 "
            f"(got {observed_azimuths})"
        )
