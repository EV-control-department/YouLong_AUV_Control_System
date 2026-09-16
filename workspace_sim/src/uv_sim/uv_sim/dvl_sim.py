"""DVL adapter: Stonefish's ``/auv/dvl`` -> ``/sim/sensor/dvl`` (``uv_msgs/Dvl``).

This is the **AQUA-SLAM adapter** of the roadmap: the message the SLAM port (T6)
consumes carries four along-beam speeds, because its Doppler pre-integration works
on ``E @ v`` per beam rather than on a fused velocity.

What it does, and why each part is a decision rather than plumbing:

* **Velocity passthrough.**  The DVL is mounted with ``rpy = 0`` (after T1), so
  Stonefish's velocity is already the body-frame velocity *at the DVL acoustic
  centre* -- including the lever arm ``omega x r_dvl`` with ``r_dvl = (-0.375, 0,
  0.1)``.  That value is deliberately **not** the same as ``/zit6/state/vel``,
  which ``nav_sim`` fills with the velocity of the body **origin**; the two differ
  by ``0.375 * |omega_z|`` (up to ~0.13 m/s), which is bug B19 and the reason
  ``check_dvl_frame.py`` used to report a non-zero ``stv err``.
* **Beam synthesis.**  ``/auv/dvl`` never fills its ``beams`` array
  (``ROS2Interface.cpp:197-216``), so the four along-beam speeds are computed here
  from the geometry the scene and the vehicle share: elevation ``alpha = 60 deg``
  from the horizontal (scene ``beam_angle = 30 deg`` off vertical) and azimuths
  ``45/135/225/315``.  This is an idealisation -- no per-beam noise -- stated in
  the contract rather than hidden: a beam-level noise model belongs to T6's config,
  not to a passthrough node.
* **Validity from altitude, not covariance.**  ``ROS2Interface.cpp:211-214`` sets
  ``velocity_covariance[0/4/8] = sigma^2 > 0`` unconditionally, so "covariance >= 0"
  would mark every sample valid.  ``msg.altitude`` is ``-1.0`` exactly when the DVL
  did not track the bottom (status != 0/2); that is the engine's own way of saying
  "no lock", and it matches the criterion ``check_dvl_frame.py`` already uses.
  When there is no bottom lock the published velocity is a *water-layer* value, so
  consumers must drop the sample when ``velocity_valid`` is false.
"""

from __future__ import annotations

import math

import numpy as np
from rclpy.node import Node

from stonefish_ros2.msg import DVL as SimDvl
from uv_msgs.msg import Dvl

from uv_sim import runner
from uv_sim.geometry import (
    DEFAULT_BEAM_AZIMUTHS_DEG,
    XUNYUN_BEAM_ANGLE_DEG,
    alpha_from_beam_angle,
    dvl_beam_directions,
)

#: Scene ``<specs beam_angle>``: angle between each beam and the sensor z axis.
DVL_BEAM_ANGLE_DEG = XUNYUN_BEAM_ANGLE_DEG

#: Beam azimuths in the sensor frame, fixed by the engine and matched by the head.
DVL_BEAM_AZIMUTHS_DEG = DEFAULT_BEAM_AZIMUTHS_DEG

#: AQUA-SLAM's ``alpha``: beam elevation from the horizontal (90 - beam_angle).
#: Feeding the off-vertical 30 deg here instead of the elevation 60 deg would scale
#: horizontal velocity by cos(30)/cos(60) = +73% -- silently (docs/frames).
DVL_ALPHA_RAD = float(alpha_from_beam_angle(math.radians(DVL_BEAM_ANGLE_DEG)))

#: 4x3 beam matrix ``E``; row i is the unit direction of beam i in the sensor frame.
DVL_BEAM_MATRIX = dvl_beam_directions(DVL_ALPHA_RAD, DVL_BEAM_AZIMUTHS_DEG)


def velocity_valid_from_altitude(altitude: float) -> bool:
    """True when the DVL tracked the bottom.

    See the module docstring for why this is not derived from
    ``velocity_covariance`` (which is always positive in the engine).
    """
    return bool(altitude >= 0.0)


def beam_velocities(vector_xyz, beam_matrix=None) -> list[float]:
    """Return the four along-beam speeds for a body-frame velocity."""
    matrix = DVL_BEAM_MATRIX if beam_matrix is None else beam_matrix
    return [float(value) for value in matrix @ np.asarray(vector_xyz, dtype=float)]


class DvlSimNode(Node):
    def __init__(self, node_name: str = 'dvl_sim', **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.publisher = self.create_publisher(Dvl, '/sim/sensor/dvl', 10)
        self.create_subscription(SimDvl, '/auv/dvl', self._dvl_cb, 10)

        self.count = 0
        self.invalid_count = 0
        self.get_logger().info(
            f"DVL adapter: beam angle {DVL_BEAM_ANGLE_DEG:.1f} deg off vertical "
            f"(AQUA-SLAM alpha {math.degrees(DVL_ALPHA_RAD):.2f} deg), "
            f"azimuths {list(DVL_BEAM_AZIMUTHS_DEG)}"
        )

    def _dvl_cb(self, msg: SimDvl) -> None:
        velocity = (msg.velocity.x, msg.velocity.y, msg.velocity.z)

        out = Dvl()
        out.header = msg.header
        out.velocity.x = velocity[0]
        out.velocity.y = velocity[1]
        out.velocity.z = velocity[2]
        out.altitude = msg.altitude
        out.velocity_valid = velocity_valid_from_altitude(msg.altitude)
        out.beam_velocity = beam_velocities(velocity)

        self.publisher.publish(out)
        self.count += 1
        if not out.velocity_valid:
            self.invalid_count += 1


def main(args=None) -> int:
    return runner.run(DvlSimNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
