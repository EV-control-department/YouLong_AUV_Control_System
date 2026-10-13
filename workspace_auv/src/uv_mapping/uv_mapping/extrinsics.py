"""The line-laser mount: one external YAML, found the way ``pid_config`` finds its file.

``T_b_line_laser`` is the difference between a bathymetry map and a shifted one:
forgetting the 19 cm translation is a fixed depth bias (the GUI's
``visualization/auv_visualizer._LASER_R_BS`` deliberately applies only the
rotation, which is fine for a profile plot and wrong for a map).  So the value
lives in a file, not in code, and the file's numbers are pinned against
``docs/frames/extrinsics_reference.md`` by ``test/test_extrinsics.py``.

Path resolution copies ``uv_sim/pid_config.resolve_pid_config_path`` on purpose
(see the long comment there): walking *up* from the module instead of using
``Path(__file__).parent.parent``, because after a plain ``colcon build`` the
package is a **copy** under ``site-packages/`` and the naive spelling silently
resolves to nothing.  Source tree wins, package share is the fallback.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from uv_mapping.geometry import se3

CONFIG_NAME = 'laser_extrinsics.yaml'
PACKAGE_NAME = 'uv_mapping'
KEY = 'T_b_line_laser'


def resolve_extrinsics_path(explicit: str = "") -> Optional[Path]:
    """Explicit parameter > source tree > package share; ``None`` if nowhere."""
    if explicit:
        candidate = Path(explicit).expanduser()
        return candidate if candidate.is_file() else None

    for parent in Path(__file__).resolve().parents:
        candidate = parent / 'config' / CONFIG_NAME
        if parent.name == PACKAGE_NAME and candidate.is_file():
            return candidate

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = (Path(get_package_share_directory(PACKAGE_NAME))
                     / 'config' / CONFIG_NAME)
        if installed.is_file():
            return installed
    except Exception:  # ament index unavailable (bare pytest run)
        pass
    return None


def load_extrinsics(path: Optional[Path] = None,
                    explicit: str = "") -> tuple[np.ndarray, dict]:
    """Return ``(T_b_laser, meta)``; raises when the file or the key is missing.

    No built-in default on purpose.  A hard-coded fallback would let the mapper
    keep running with a mount that disagrees with the scene, which is exactly the
    silent-scale-error failure mode B16 (wrong DVL alpha, +31 % horizontal speed)
    and B21 (ignored frame bit) already cost this project.  Loud is cheaper.
    """
    resolved = path if path is not None else resolve_extrinsics_path(explicit)
    if resolved is None or not Path(resolved).is_file():
        raise FileNotFoundError(
            f"{CONFIG_NAME} not found (source tree, then package share of "
            f"{PACKAGE_NAME}); pass extrinsics_file:=/path/to/{CONFIG_NAME}")

    resolved = Path(resolved)
    with resolved.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict) or KEY not in data:
        raise ValueError(f"{resolved} has no '{KEY}' key")

    matrix = np.asarray(data[KEY], dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError(f"{resolved}: '{KEY}' must be 4x4, got {matrix.shape}")
    rotation = matrix[:3, :3]
    # 1e-4, not 1e-9: the generated mount carries the engine's own yaw residual
    # (3.673e-06 in the off-diagonal), and ``derive_extrinsics.py --check``
    # compares element-wise, so rounding it away here would reject the real
    # value.  A genuine mistake (a reflection, a scale factor, a transposed
    # block) is orders of magnitude larger.
    if not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-4):
        raise ValueError(f"{resolved}: '{KEY}' rotation block is not orthonormal")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError(f"{resolved}: '{KEY}' last row must be [0,0,0,1]")

    meta = {k: v for k, v in data.items() if k != KEY}
    meta['path'] = str(resolved)
    return matrix, meta


def laser_to_body(points_plane: np.ndarray, T_b_laser: np.ndarray | None = None
                  ) -> np.ndarray:
    """Laser-plane points (``z == 0``) -> body (FRD) points.

    Kept as a named function rather than an inline matrix multiply because the
    translation term is the part people drop.
    """
    if T_b_laser is None:
        T_b_laser, _ = load_extrinsics()
    pts = np.asarray(points_plane, dtype=float)
    if pts.size == 0:
        return pts.reshape(0, 3)
    return pts @ T_b_laser[:3, :3].T + T_b_laser[:3, 3]


def body_to_world(points_body: np.ndarray, T_world_body: np.ndarray) -> np.ndarray:
    """Body (FRD) points -> map/world (NED) points."""
    pts = np.asarray(points_body, dtype=float)
    if pts.size == 0:
        return pts.reshape(0, 3)
    return pts @ T_world_body[:3, :3].T + T_world_body[:3, 3]


def extrinsics_matrix(values) -> np.ndarray:
    """Build a 4x4 from a nested list -- convenience for tests and fixtures."""
    return se3(np.asarray(values, dtype=float)[:3, :3],
               np.asarray(values, dtype=float)[:3, 3])
