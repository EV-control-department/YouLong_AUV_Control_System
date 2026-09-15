"""Frame conventions and DVL beam geometry (single source of truth).

This module is the executable half of ``docs/frames/``.  It is deliberately
free of ROS imports so the unit tests and the ``tools/`` checks run without a
sourced ROS environment.

Conventions (see ``docs/frames/frame_convention.md`` for the code anchors):

* **world_ned** -- North-East-Down.  ``x=North``, ``y=East``, ``z=Down``;
  ``z=0`` is the water surface and positive ``z`` is deeper.
* **body** (``<robot>/base_link``) -- ``x=forward`` (bow), ``y=starboard``,
  ``z=down``.  ``yaw=0`` points the bow at North.
* **rpy composition** -- Stonefish parses ``<origin rpy="r p y">`` as
  ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`` (``ScenarioParser.cpp`` builds
  ``btQuaternion(rpy.z, rpy.y, rpy.x)`` and ``btQuaternion`` uses
  ``setEulerZYX``).  :func:`euler_rpy_to_matrix` implements exactly that.

DVL beam angles -- **two conventions, one trap**:

* ``theta`` -- the *scene* quantity ``<specs beam_angle>``: the angle between a
  beam and the sensor **-z/+z axis**, i.e. off **vertical**.  The XUNYUN DVL is
  ``theta = 30 deg``.
* ``alpha`` -- the *AQUA-SLAM* quantity: the beam **elevation from the
  horizontal plane**, so ``alpha = 90 deg - theta = 60 deg``.  It feeds
  ``mE`` in ``AQUA-SLAM/src/DVLGroPreIntegration.cpp:54`` whose rows are
  ``(-cos(beta)*cos(alpha), sin(beta)*cos(alpha), sin(alpha))``.

Feeding ``theta`` where ``alpha`` is expected is silent: horizontal velocity
comes out scaled by ``cos(30)/cos(60) = 1.732`` (+73%), and the AQUA-SLAM
shipped default ``alpha = 67.5 deg`` gives ``+30.7%``.  Use
:func:`alpha_from_beam_angle` / :func:`beam_angle_from_alpha` instead of
converting by hand, and :func:`beam_inversion_sensitivity` to print the cost.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

#: Four-beam DVL azimuths, in the sensor frame.  Stonefish fixes them at
#: ``45 deg + i*90 deg`` (``sf::DVL``); the real XUNYUN head uses the same
#: layout, so the sim and the vehicle agree without a yaw mount offset.
DEFAULT_BEAM_AZIMUTHS_DEG: tuple[float, float, float, float] = (45.0, 135.0, 225.0, 315.0)

#: ``<specs beam_angle>`` of the XUNYUN DVL: 30 deg off vertical.
XUNYUN_BEAM_ANGLE_DEG = 30.0

#: Sensor names in ``xunyun_fixed.scn`` that are of interest to the frame docs.
SCENE_SENSOR_NAMES: tuple[str, ...] = (
    "dynamics",
    "pressure",
    "dvl",
    "imu_filter",
    "front_cam_left",
    "front_cam_right",
    "down_cam_left",
    "down_cam_right",
)

#: Default scene used by the tools (DVL/IMU/camera extrinsics baseline).
DEFAULT_SCENE = "workspace_sim/src/stonefish_ros2/Data/xunyun_fixed.scn"


# --------------------------------------------------------------------------- #
# Rotations and rigid transforms
# --------------------------------------------------------------------------- #
def euler_rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` (Stonefish ``<origin rpy>``).

    Angles are in **radians**.  This is the only euler-order helper: do not
    hand-roll ``Rx*Ry*Rz`` elsewhere, the two differ unless yaw is the only
    non-zero angle.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def transform_from_xyz_rpy(
    xyz: tuple[float, float, float] | list[float] | np.ndarray,
    rpy: tuple[float, float, float] | list[float] | np.ndarray,
) -> np.ndarray:
    """Build a 4x4 homogeneous transform from a translation and ZYX euler rpy."""
    transform = np.eye(4)
    transform[:3, :3] = euler_rpy_to_matrix(float(rpy[0]), float(rpy[1]), float(rpy[2]))
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


def invert_rigid(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid 4x4 transform as ``[R^T | -R^T t]``."""
    transform = np.asarray(transform, dtype=float)
    if transform.shape != (4, 4):
        raise ValueError(f"expected a 4x4 transform, got {transform.shape}")
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def check_rotation(matrix: np.ndarray, tolerance: float = 1e-9) -> bool:
    """True when ``matrix`` is a proper rotation (orthonormal, det=+1)."""
    matrix = np.asarray(matrix, dtype=float)
    return bool(
        np.allclose(matrix.T @ matrix, np.eye(3), atol=tolerance)
        and abs(np.linalg.det(matrix) - 1.0) < tolerance
    )


# --------------------------------------------------------------------------- #
# DVL beam geometry
# --------------------------------------------------------------------------- #
def alpha_from_beam_angle(beam_angle_rad: float | np.ndarray) -> float | np.ndarray:
    """AQUA-SLAM ``alpha`` (elevation from horizontal) from the scene angle.

    ``<specs beam_angle>`` measures off vertical, so ``alpha = pi/2 - theta``.
    """
    return math.pi / 2.0 - np.asarray(beam_angle_rad, dtype=float)


def beam_angle_from_alpha(alpha_rad: float | np.ndarray) -> float | np.ndarray:
    """Inverse of :func:`alpha_from_beam_angle` (off-vertical scene angle)."""
    return math.pi / 2.0 - np.asarray(alpha_rad, dtype=float)


def dvl_beam_directions(
    alpha_rad: float | np.ndarray,
    beta_rad: np.ndarray | list[float] | tuple[float, ...] = DEFAULT_BEAM_AZIMUTHS_DEG,
    betas_are_degrees: bool = True,
) -> np.ndarray:
    """Return the 4x3 DVL beam matrix ``E`` used by AQUA-SLAM.

    Row ``i`` is the unit direction of beam ``i`` in the **sensor** frame::

        E[i] = (-cos(beta_i)*cos(alpha_i), sin(beta_i)*cos(alpha_i), sin(alpha_i))

    matching ``AQUA-SLAM/src/DVLGroPreIntegration.cpp:54`` (``mE``).  With
    ``alpha = 60 deg`` and azimuths ``45/135/225/315`` the beams point 30 deg
    off the sensor's +z axis (down in NED).  The overall sign of ``E`` is
    irrelevant to :func:`beam_velocity_to_body`; only the geometry matters.
    """
    alpha = np.asarray(alpha_rad, dtype=float)
    beta = np.asarray(beta_rad, dtype=float)
    if betas_are_degrees:
        beta = np.radians(beta)
    beta = np.atleast_1d(beta)
    if beta.size == 1:
        beta = np.repeat(beta, 4)
    if beta.size != 4:
        raise ValueError(f"expected 4 beam azimuths, got {beta.size}")
    alpha = np.broadcast_to(alpha, beta.shape).astype(float)
    cos_a, sin_a = np.cos(alpha), np.sin(alpha)
    return np.stack(
        (-np.cos(beta) * cos_a, np.sin(beta) * cos_a, sin_a), axis=1
    )


def beam_velocity_to_body(beam_matrix: np.ndarray, beam_velocity: np.ndarray) -> np.ndarray:
    """Least-squares DVL inversion ``v_body = (E^T E)^-1 E^T v_beam``.

    ``beam_velocity`` holds the four along-beam speeds.  Exact (to machine
    precision) whenever ``E`` has rank 3 -- the solver does *not* need
    orthogonal or unit columns, so four arbitrary azimuths are fine.
    """
    beam_matrix = np.asarray(beam_matrix, dtype=float)
    beam_velocity = np.asarray(beam_velocity, dtype=float).reshape(-1)
    if beam_matrix.shape[1] != 3:
        raise ValueError(f"E must be Nx3, got {beam_matrix.shape}")
    if beam_matrix.shape[0] != beam_velocity.size:
        raise ValueError(
            f"E has {beam_matrix.shape[0]} beams but v_beam has {beam_velocity.size}"
        )
    gram = beam_matrix.T @ beam_matrix
    return np.linalg.solve(gram, beam_matrix.T @ beam_velocity)


def beam_inversion_sensitivity(
    alpha_assumed_rad: float,
    alpha_true_rad: float = math.radians(60.0),
    beta_rad: np.ndarray | list[float] | tuple[float, ...] = DEFAULT_BEAM_AZIMUTHS_DEG,
) -> tuple[float, float]:
    """Horizontal/vertical scale factors from using the wrong ``alpha``.

    Returns ``(horizontal_scale, vertical_scale)``: the factor multiplying a
    true unit surge (respectively heave) when the beams are inverted with
    ``alpha_assumed`` while the sensor actually has ``alpha_true``.  A value of
    1.0 means no error; ``0.6934`` means the reported horizontal speed is 31%
    low, ``1.3066`` means 31% high.
    """
    beams_true = dvl_beam_directions(alpha_true_rad, beta_rad)
    beams_assumed = dvl_beam_directions(alpha_assumed_rad, beta_rad)
    horizontal = beam_velocity_to_body(beams_assumed, beams_true @ np.array([1.0, 0.0, 0.0]))[0]
    vertical = beam_velocity_to_body(beams_assumed, beams_true @ np.array([0.0, 0.0, 1.0]))[2]
    return float(horizontal), float(vertical)


# --------------------------------------------------------------------------- #
# .scn parsing
# --------------------------------------------------------------------------- #
@dataclass
class SceneSensor:
    """One ``<sensor>``/``<comm>`` block of a Stonefish scene file."""

    name: str
    kind: str  # the ``type=`` attribute: odometry, dvl, imu, camera, pressure, usbl
    link: str = ""
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rate_hz: float | None = None
    specs: dict[str, str] = field(default_factory=dict)
    ros_topics: dict[str, str] = field(default_factory=dict)
    noise: dict[str, str] = field(default_factory=dict)

    @property
    def transform_body_to_sensor(self) -> np.ndarray:
        """``T_b_s``: sensor pose expressed in the body frame."""
        return transform_from_xyz_rpy(self.xyz, self.rpy)

    @property
    def transform_sensor_to_body(self) -> np.ndarray:
        """``T_s_b``: rigid inverse of :attr:`transform_body_to_sensor`."""
        return invert_rigid(self.transform_body_to_sensor)

    @property
    def beam_angle_rad(self) -> float | None:
        """``<specs beam_angle>`` in radians, or ``None`` for non-DVL sensors.

        The scene file writes the attribute in **degrees** (``30.0``), while
        most Stonefish angles (``<origin rpy>``) are radians.  This property
        hides that inconsistency; never read ``specs['beam_angle']`` directly.
        """
        raw = self.specs.get("beam_angle")
        return None if raw is None else math.radians(float(raw))

    @property
    def alpha_rad(self) -> float | None:
        """AQUA-SLAM ``alpha`` (elevation from horizontal) for a DVL sensor."""
        beam_angle = self.beam_angle_rad
        return None if beam_angle is None else float(alpha_from_beam_angle(beam_angle))


def _floats(text: str | None, count: int) -> tuple[float, ...] | None:
    if text is None:
        return None
    values = tuple(float(part) for part in text.split())
    if len(values) != count:
        raise ValueError(f"expected {count} numbers, got {len(values)!r} in {text!r}")
    return values


def parse_scn_sensors(path: str | Path) -> dict[str, SceneSensor]:
    """Read every ``<sensor>`` and ``<comm>`` block of a ``.scn`` file.

    Only the data the frame documentation needs is extracted: ``name``,
    ``type``, ``<link>``, ``<origin xyz/rpy>``, the ``rate`` attribute,
    ``<specs>`` attributes, ``<noise>`` attributes and the ``<ros_publisher>``
    topics.  Unknown attributes are ignored, so a scene may carry extra blocks
    without breaking the parser.
    """
    root = ET.parse(str(path)).getroot()
    sensors: dict[str, SceneSensor] = {}
    for element in root.iter():
        if element.tag not in ("sensor", "comm"):
            continue
        name = element.get("name")
        if not name:
            continue
        sensor = SceneSensor(name=name, kind=element.get("type", element.tag))
        raw_rate = element.get("rate")
        if raw_rate is not None:
            sensor.rate_hz = float(raw_rate)
        origin = element.find("origin")
        if origin is not None:
            xyz = _floats(origin.get("xyz"), 3)
            rpy = _floats(origin.get("rpy"), 3)
            if xyz is not None:
                sensor.xyz = xyz
            if rpy is not None:
                sensor.rpy = rpy
        link = element.find("link")
        if link is not None and link.get("name"):
            sensor.link = link.get("name", "")
        specs = element.find("specs")
        if specs is not None:
            sensor.specs = dict(specs.attrib)
        noise = element.find("noise")
        if noise is not None:
            sensor.noise = dict(noise.attrib)
        publisher = element.find("ros_publisher")
        if publisher is not None:
            for key, value in publisher.attrib.items():
                sensor.ros_topics[key] = value
        # A DVL with no explicit publisher still has a conventional altitude topic;
        # only record what the scene declares so the docs stay evidence-based.
        sensors[name] = sensor
    return sensors


def dvl_beam_azimuths_from_scene(sensor: SceneSensor) -> tuple[float, ...]:
    """Beam azimuths of a scene DVL, from the engine's fixed ``45 + i*90`` layout.

    Returns degrees.  Kept as a function (not a constant) so the day Stonefish
    stops hard-coding the layout there is exactly one place to change.
    """
    if sensor.kind != "dvl":
        raise ValueError(f"sensor {sensor.name!r} is a {sensor.kind!r}, not a dvl")
    return DEFAULT_BEAM_AZIMUTHS_DEG
