"""SE(3) and pose-interpolation maths for the mapping stack.  numpy only, no ROS.

Why a copy of maths that also exists in ``uv_sim/geometry.py`` and
``uv_control/coordinate.py``: this package lives in ``workspace_auv`` (the real
vehicle stack) and must not import from ``workspace_sim``, and a mapper that
cannot be unit-tested without a node is a mapper whose frame conventions nobody
ever checks.  The conventions themselves are not re-invented here -- the single
authority is ``docs/frames/frame_convention.md`` and
``docs/frames/extrinsics_reference.md``, and ``test/test_geometry.py`` pins the
few numbers that would silently flip a sign:

* NED world, FRD body: ``x`` north/forward, ``y`` east/starboard, ``z`` down.
* ``rpy_to_matrix`` is ``R = Rz(yaw) . Ry(pitch) . Rx(roll)`` -- the *only*
  composition used anywhere in this project.  The plausible wrong version
  (``Rx . Ry . Rz``) agrees for pure-yaw poses, which is why the unit test
  deliberately uses a mixed attitude.
* Yaw is clockwise-positive about +z (down), i.e. ``yaw = +90 deg`` maps the
  body +x axis onto world +y (east).  Getting this backwards mirrors the whole
  map and is invisible in a plot.

Angles are **radians** at this layer.  Degrees live at the ROS/config boundary
(``PoseInfo`` writes degrees, ``docs/frames/frame_convention.md`` says so).
"""

from __future__ import annotations

import math

import numpy as np


def rpy_to_matrix(roll_rad: float, pitch_rad: float, yaw_rad: float) -> np.ndarray:
    """``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``, the project's one convention."""
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """``(x, y, z, w)`` quaternion from a rotation matrix (Shepperd's method).

    Branching on the largest diagonal term keeps the square root away from zero,
    which the naive trace formula does not: a 180 deg rotation has trace -1 and
    the naive version divides by ~0.
    """
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return np.array([(m[2, 1] - m[1, 2]) / s,
                         (m[0, 2] - m[2, 0]) / s,
                         (m[1, 0] - m[0, 1]) / s,
                         0.25 * s])
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return np.array([0.25 * s,
                         (m[0, 1] + m[1, 0]) / s,
                         (m[0, 2] + m[2, 0]) / s,
                         (m[2, 1] - m[1, 2]) / s])
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return np.array([(m[0, 1] + m[1, 0]) / s,
                         0.25 * s,
                         (m[1, 2] + m[2, 1]) / s,
                         (m[0, 2] - m[2, 0]) / s])
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return np.array([(m[0, 2] + m[2, 0]) / s,
                     (m[1, 2] + m[2, 1]) / s,
                     0.25 * s,
                     (m[1, 0] - m[0, 1]) / s])


def quat_to_matrix(q) -> np.ndarray:
    """``(x, y, z, w)`` -> rotation matrix (input is normalised here)."""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_slerp(q0, q1, alpha: float) -> np.ndarray:
    """Shortest-arc spherical interpolation; falls back to lerp for tiny angles."""
    a = np.asarray(q0, dtype=float)
    b = np.asarray(q1, dtype=float)
    dot = float(np.dot(a, b))
    if dot < 0.0:               # q and -q are the same rotation; take the short way
        b, dot = -b, -dot
    if dot > 0.9995:
        out = a + alpha * (b - a)
        return out / np.linalg.norm(out)
    theta0 = math.acos(max(-1.0, min(1.0, dot)))
    theta = theta0 * alpha
    tangent = b - a * dot
    norm = np.linalg.norm(tangent)
    if norm < 1e-12:
        return a
    out = a * math.cos(theta) + (tangent / norm) * math.sin(theta)
    return out / np.linalg.norm(out)


def se3(R: np.ndarray | None = None, t=None) -> np.ndarray:
    """4x4 homogeneous transform from a rotation and a translation."""
    out = np.eye(4)
    if R is not None:
        out[:3, :3] = R
    if t is not None:
        out[:3, 3] = np.asarray(t, dtype=float)
    return out


def invert_se3(T: np.ndarray) -> np.ndarray:
    """Rigid inverse: ``R^T`` and ``-R^T t`` -- never ``np.linalg.inv``."""
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def transform_points(T: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply ``T`` to an ``(N, 3)`` array of points."""
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return pts.reshape(0, 3)
    return pts @ T[:3, :3].T + T[:3, 3]


def nearest_index(stamps: np.ndarray, query: np.ndarray) -> np.ndarray:
    """Index of the closest stamp for each query time (vectorised)."""
    stamps = np.asarray(stamps, dtype=float)
    query = np.asarray(query, dtype=float)
    if stamps.size == 0:
        raise ValueError("nearest_index: empty stamp table")
    idx = np.searchsorted(stamps, query).clip(1, stamps.size - 1)
    left = stamps[idx - 1]
    right = stamps[np.minimum(idx, stamps.size - 1)]
    return np.where(np.abs(query - left) <= np.abs(query - right),
                    idx - 1, np.minimum(idx, stamps.size - 1))


def nearest_delta(stamps: np.ndarray, query: np.ndarray) -> np.ndarray:
    """``|t_query - t_nearest|`` for each query time [same unit as the input]."""
    stamps = np.asarray(stamps, dtype=float)
    if stamps.size == 0:
        return np.full(np.asarray(query).shape, np.inf)
    return np.abs(np.asarray(query, dtype=float) - stamps[nearest_index(stamps, query)])


def interpolate_poses(stamps: np.ndarray, poses: np.ndarray,
                      query: np.ndarray) -> np.ndarray:
    """Poses at arbitrary times: linear position + SLERP rotation.

    **This is not a nicety, it is the difference between a clean map and a
    comb.**  Snapping every laser strip to the nearest pose sample quantises the
    *attitude* too, and attitude error is amplified by range: at 14 deg/s of yaw
    a 30 Hz pose stream steps 0.47 deg per strip, which at 5 m range is 4 cm.  A
    recorded survey showed exactly that -- the cross-strip histogram grew teeth
    at 0, 1, 2 and 3.5 cm that vanished once the pose was interpolated to the
    strip's own timestamp.

    Query times outside the table clamp to the first/last pose (a strip with no
    bracketing pose must not silently become a zero translation).
    """
    stamps = np.asarray(stamps, dtype=float)
    poses = np.asarray(poses, dtype=float)
    query = np.asarray(query, dtype=float).ravel()
    if stamps.size == 0 or poses.size == 0:
        raise ValueError("interpolate_poses: empty pose table")
    if stamps.size == 1:
        return np.repeat(poses[:1], query.size, axis=0)

    quats = np.array([matrix_to_quat(poses[i][:3, :3]) for i in range(len(stamps))])
    j = np.searchsorted(stamps, query).clip(1, len(stamps) - 1)
    t0 = stamps[j - 1]
    t1 = stamps[j]
    span = np.where(t1 > t0, t1 - t0, 1.0)
    alpha = np.clip((query - t0) / span, 0.0, 1.0)

    out = np.zeros((query.size, 4, 4))
    for i in range(query.size):
        k = int(j[i])
        a = float(alpha[i])
        out[i] = se3(quat_to_matrix(quat_slerp(quats[k - 1], quats[k], a)),
                     poses[k - 1][:3, 3] * (1.0 - a) + poses[k][:3, 3] * a)
    return out
