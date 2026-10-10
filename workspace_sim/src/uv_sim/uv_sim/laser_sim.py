"""Line-laser adapter: Stonefish's ``/auv/laser_scan`` -> ``/sim/sensor/laser_points``.

The vehicle scene declares the line laser as a ``type="multibeam"`` sensor
(``xunyun_fixed.scn``, sensor ``line_laser``): **one plane of rays, i.e. one
line** projected onto the seabed.  The engine publishes that as a
``sensor_msgs/LaserScan`` -- ranges only, polar, in the *sensor* frame.  This
node turns it into what a seabed-reconstruction pipeline actually wants: the 3-D
positions of the points where the line hits the bottom.

Each decision below has a plausible wrong version, which is why it is written
down instead of left to the code:

* **The sensor frame, not the body frame.**  ``frame_id`` is forwarded from the
  scan (the engine writes the *sensor name*, ``line_laser``).  The points then
  lie in the laser plane, ``z = 0``, which is exactly what a structured-light
  measurement *is*: a 2-D profile plus a known plane.  Lifting it into the body
  or world frame needs the mount (``config/extrinsics/xunyun.yaml``, generated
  from the scene by ``tools/derive_extrinsics.py``) and the vehicle pose; the
  node hard-codes neither.  There is no TF in this simulation at all
  (``docs/frames/frame_convention.md`` section 3), so consumers read the YAML.
* **Angles come from the two endpoints.**  ``ROS2Interface.cpp`` writes
  ``angle_min = -fov/2``, ``angle_max = +fov/2`` and ``N = steps + 1`` samples
  **including both ends**, so the spacing is
  ``(angle_max - angle_min) / (N - 1)``.  ``linspace`` states that directly and
  stays correct even if the redundant ``angle_increment`` field is ever written
  with a different convention.
* **Validity is the engine's own flag.**  ``PublishMultibeam`` sets
  ``ranges[i] = inf`` *and* ``intensities[i] = 0`` exactly when the ray missed
  (``Multibeam::InternalUpdate`` writes ``rangeMax`` on a miss and the publisher
  compares against it).  A miss is not a point 15 m away, so it is dropped here
  instead of published.  ``intensities`` is the primary source because it is the
  engine's explicit marker; ``isfinite`` is the fallback for a message that
  omits it.
* **One topic, and it is ours.**  ``/auv/laser_scan`` is the engine's;
  ``/sim/sensor/laser_points`` is ours, inside the ``/sim/sensor/*`` namespace
  the T5 split owns.  Nothing consumes it in the SIL stack yet -- registered as
  a gap in ``docs/interfaces/topic_contract.md`` section 9, like
  ``/sim/sensor/pressure``.

The sensor is mounted looking straight down with the line along the body
``y`` axis (FRD), so the profile this publishes is what the vehicle flies over.
"""

from __future__ import annotations

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

from uv_sim import runner

# Topics: ``/auv/laser_scan`` (the engine's, from the scene's
# ``<ros_publisher>``) -> ``/sim/sensor/laser_points`` (ours).  Written as string
# literals at the ``create_*`` calls on purpose: ``scripts/check_docs_contract.py``
# resolves literal topic strings only, and every other ``uv_sim`` node does the
# same -- a module-level constant shows up as ``<dynamic:...>`` and fails the
# contract check instead of being silently ignored.


def scan_angles(scan: LaserScan) -> np.ndarray:
    """Angle of every beam [rad], sensor ``x`` axis = 0, toward sensor ``y``.

    See the module docstring: the engine's samples include both endpoints.
    """
    count = len(scan.ranges)
    if count == 0:
        return np.empty(0, dtype=float)
    if count == 1:
        return np.array([float(scan.angle_min)], dtype=float)
    return np.linspace(float(scan.angle_min), float(scan.angle_max), count)


def valid_mask(scan: LaserScan) -> np.ndarray:
    """True where the beam came back off the bottom.

    ``PublishMultibeam`` writes ``intensity = 0`` (and ``range = inf``) for a
    miss; both are checked so that neither a stale engine nor an
    ``intensity``-less message can turn a miss into a point.
    """
    ranges = np.asarray(scan.ranges, dtype=float)
    ok = np.isfinite(ranges) & (ranges > 0.0)
    intensities = np.asarray(scan.intensities, dtype=float)
    if intensities.shape == ranges.shape:
        ok &= intensities > 0.0
    return ok


def scan_to_points(scan: LaserScan) -> np.ndarray:
    """Valid returns as an ``(M, 3)`` array in the sensor frame.

    Same convention as the engine's own ``PublishMultibeamPCL``: the fan lies in
    the sensor ``xy`` plane, so ``z = 0`` for every point.
    """
    angles = scan_angles(scan)
    ranges = np.asarray(scan.ranges, dtype=float)
    valid = valid_mask(scan)
    if not valid.any():
        return np.empty((0, 3), dtype=float)
    hit_angles = angles[valid]
    hit_ranges = ranges[valid]
    return np.stack(
        [
            np.cos(hit_angles) * hit_ranges,
            np.sin(hit_angles) * hit_ranges,
            np.zeros_like(hit_angles),
        ],
        axis=1,
    )


class LaserSimNode(Node):
    def __init__(self, node_name: str = 'laser_sim', **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.publisher = self.create_publisher(PointCloud2, '/sim/sensor/laser_points', 10)
        self.create_subscription(LaserScan, '/auv/laser_scan', self._scan_cb, 10)

        self.count = 0
        self.missed = 0     # beams that did not come back, summed over scans
        self.get_logger().info(
            "line laser: /auv/laser_scan -> /sim/sensor/laser_points "
            "(stripe points in the sensor/laser-plane frame)"
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        points = scan_to_points(msg)

        cloud = point_cloud2.create_cloud_xyz32(msg.header, points.tolist())
        # Every published point came back off the bottom, so the cloud has no
        # invalid entries -- create_cloud_xyz32 cannot know that.
        cloud.is_dense = True
        self.publisher.publish(cloud)

        self.count += 1
        self.missed += len(msg.ranges) - len(points)


def main(args=None) -> int:
    return runner.run(LaserSimNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
