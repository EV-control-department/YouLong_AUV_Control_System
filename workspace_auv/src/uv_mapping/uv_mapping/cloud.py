"""``sensor_msgs/PointCloud2`` -> ``(N, 3)`` numpy, without importing sensor_msgs.

The decoder is duck-typed on ``fields`` / ``width`` / ``height`` / ``point_step``
/ ``data`` so the core stays ROS-free and unit-testable, and so the same code
serves the node, the offline replay and a plain pytest run.

Two details that are easy to get wrong and are therefore explicit here:

* **Stride, not packing.**  ``point_step`` may be larger than the extent of the
  x/y/z fields (the engine's PCL writers pad to 16 or 32 bytes; the engine's own
  ``create_cloud_xyz32`` happens to use 12).  Parsing with a structured dtype
  whose ``itemsize`` is ``point_step`` reads the right bytes either way, whereas
  ``np.frombuffer(data, '<f4').reshape(-1, 3)`` only works for the packed case.
* **Validity is the caller's business.**  ``laser_sim`` already drops the rays
  that missed the seabed, so a point here is a real return.  Nothing in this
  function filters; if a producer ever starts emitting ``NaN`` for a miss, that
  belongs in a named predicate, not in a silent ``isfinite`` here.
"""

from __future__ import annotations

import numpy as np


def field_offsets(msg) -> dict:
    """``{name: byte offset}`` for the message's fields."""
    return {field.name: field.offset for field in msg.fields}


def decode_xyz(msg) -> np.ndarray:
    """Point positions as an ``(N, 3)`` float64 array.

    Returns an empty array for an empty cloud; raises ``ValueError`` when the
    cloud declares no x/y/z or the buffer is shorter than the cloud claims --
    a short buffer means the message is corrupt, and silently returning fewer
    points would look exactly like "the laser saw less".
    """
    offsets = field_offsets(msg)
    missing = {"x", "y", "z"} - set(offsets)
    if missing:
        raise ValueError(f"PointCloud2 has no {sorted(missing)} field(s)")
    count = int(msg.width) * int(msg.height)
    if count == 0:
        return np.zeros((0, 3), dtype=float)

    step = int(msg.point_step)
    largest = max(offsets["x"], offsets["y"], offsets["z"])
    payload = bytes(msg.data)
    if len(payload) < (count - 1) * step + largest + 4:
        raise ValueError(
            f"PointCloud2 buffer is {len(payload)} bytes but claims {count} points "
            f"of {step} bytes")

    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": ["<f4", "<f4", "<f4"],
        "offsets": [offsets["x"], offsets["y"], offsets["z"]],
        "itemsize": step,
    })
    records = np.frombuffer(payload[: count * step], dtype=dtype, count=count)
    return np.stack([records["x"], records["y"], records["z"]],
                    axis=1).astype(float)


def finite_mask(points: np.ndarray) -> np.ndarray:
    """Rows that are real 3-D measurements (all three components finite)."""
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return np.zeros(0, dtype=bool)
    return np.isfinite(pts).all(axis=1)
