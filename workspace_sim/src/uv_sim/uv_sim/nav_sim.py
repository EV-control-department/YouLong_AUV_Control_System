"""Aggregate ``/sim/sensor/*`` into the two MCU-facing navigation outputs.

One node, two mutually exclusive roles (the SIL/HIL switch):

======================  ========================  ===================================
parameter               output                    who owns ``/zit6/state/*``
======================  ========================  ===================================
``publish_state``       ``/zit6/state/pos|vel``   this node (SIL: nobody else is MCU)
``publish_sim_nav``     ``/zit6/sim/nav``         the real MCU (HIL: injects truth)
======================  ========================  ===================================

Both true would put two publishers on ``/zit6/state/*`` -- invariant I2 -- so it is
a **startup error**, not a warning.  Both false is legal but useless (the node is
then a silent consumer), which is only a warning so that a direct
``ros2 run uv_sim nav_sim`` experiment is not blocked.

Not an INS: this node assembles the *navigation solution the MCU contract expects*
from ground-truth odometry and IMU.  There is no filtering, no DVL, and therefore
no drift -- and that is deliberate (``docs/architecture/sim_real_boundary.md``): the
SIL control loop must not silently depend on the DVL, which the real MCU does use.
The consequence is a **fidelity gap** documented in the topic contract: SIL's
``/zit6/state/vel`` is the velocity of the body origin taken from odometry.

Rates are kept from the pre-T5 monolith: pos 30 Hz, vel 60 Hz (both published from
one 60 Hz control timer, gated by tick), ``/zit6/sim/nav`` at the odometry rate
(30 Hz, one message per odometry sample -- upstream velocity/attitude freshness).
"""

from __future__ import annotations

import math

from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray

from uv_sim import runner
from uv_sim.odometry import (
    apply_imu_to_state,
    apply_odometry_to_state,
    body_twist_from_odometry,
    quat_to_rpy,
)

#: Control-timer period; position is published every second tick (30 Hz).
CONTROL_PERIOD_SECONDS = 0.0167
POS_EVERY_N_TICKS = 2


class NavSimNode(Node):
    def __init__(self, node_name: str = 'nav_sim', **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.declare_parameter('publish_state', True)
        self.declare_parameter('publish_sim_nav', False)
        self.publish_state = bool(self.get_parameter('publish_state').value)
        self.publish_sim_nav = bool(self.get_parameter('publish_sim_nav').value)

        if self.publish_state and self.publish_sim_nav:
            self.get_logger().error(
                "publish_state and publish_sim_nav are mutually exclusive: "
                "/zit6/state/* may have only one publisher (I2) -- in HIL the real "
                "MCU owns it, so run with publish_state:=false"
            )
            raise RuntimeError(
                "nav_sim: publish_state and publish_sim_nav cannot both be true"
            )
        if not (self.publish_state or self.publish_sim_nav):
            self.get_logger().warning(
                "both publish_state and publish_sim_nav are false: nav_sim will "
                "consume /sim/sensor/* and publish nothing"
            )

        # State in the internal convention of the control code: NED degrees / deg/s.
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        self._tick = 0
        self.pos_count = 0
        self.vel_count = 0

        # /zit6/sim/nav payload: pos_world[6] in radians + vel_body[6] in SI units.
        self._sim_pos = [0.0] * 6
        self._sim_vel = [0.0] * 6
        self.sim_nav_count = 0

        if self.publish_state:
            self.zit6_pos_pub = self.create_publisher(
                Float32MultiArray, '/zit6/state/pos', 10
            )
            self.zit6_vel_pub = self.create_publisher(
                Float32MultiArray, '/zit6/state/vel', 10
            )
            self.create_timer(CONTROL_PERIOD_SECONDS, self._tick_60hz)
        if self.publish_sim_nav:
            self.sim_nav_pub = self.create_publisher(
                Float32MultiArray, '/zit6/sim/nav', 10
            )

        self.create_subscription(Odometry, '/sim/sensor/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/sim/sensor/imu', self._imu_cb, 10)

        self.get_logger().info(
            "nav_sim started: "
            + ("publishing /zit6/state/{pos,vel}" if self.publish_state
               else "publishing /zit6/sim/nav (state topics belong to the MCU)")
        )

    # ── inputs ──────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        """Decode one odometry sample into whichever output(s) are enabled."""
        if self.publish_state:
            apply_odometry_to_state(msg, self.pos, self.vel)

        if self.publish_sim_nav:
            self._sim_pos[0] = msg.pose.pose.position.x
            self._sim_pos[1] = msg.pose.pose.position.y
            self._sim_pos[2] = msg.pose.pose.position.z
            q = msg.pose.pose.orientation
            roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
            self._sim_pos[3] = roll
            self._sim_pos[4] = pitch
            self._sim_pos[5] = yaw

            # Twist is already body-frame (see body_twist_from_odometry); p, q, r
            # are refreshed by the IMU callback.
            u, v, w, _, _, _ = body_twist_from_odometry(msg)
            self._sim_vel[0] = u
            self._sim_vel[1] = v
            self._sim_vel[2] = w

            self._publish_sim_nav()

    def _imu_cb(self, msg: Imu) -> None:
        """Refresh angular rates -- degrees/s for the state, rad/s for sim/nav."""
        if self.publish_state:
            apply_imu_to_state(msg, self.vel)
        if self.publish_sim_nav:
            self._sim_vel[3] = msg.angular_velocity.x
            self._sim_vel[4] = msg.angular_velocity.y
            self._sim_vel[5] = msg.angular_velocity.z

    # ── outputs ─────────────────────────────────────────────────────────────

    def _tick_60hz(self) -> None:
        """60 Hz control tick; position is gated to 30 Hz (vel stays 60 Hz)."""
        self._tick = (self._tick + 1) % POS_EVERY_N_TICKS
        if self._tick == 0:
            self._publish_pos()
        self._publish_vel()

    def _publish_pos(self) -> None:
        """``[x, y, z, roll_rad, pitch_rad, yaw_rad]`` (30 Hz)."""
        pos_msg = Float32MultiArray()
        pos_msg.data = [
            self.pos['x'], self.pos['y'], self.pos['z'],
            math.radians(self.pos.get('rx', 0.0)),
            math.radians(self.pos.get('ry', 0.0)),
            math.radians(self.pos['rz']),
        ]
        self.zit6_pos_pub.publish(pos_msg)
        self.pos_count += 1

    def _publish_vel(self) -> None:
        """``[vx, vy, vz, vroll_rad, vpitch_rad, vyaw_rad]``, body frame (60 Hz).

        Linear parts are the velocity of the **body origin** (odometry twist, no
        DVL lever arm): this is the single definition that fixes bug B19.
        """
        vel_msg = Float32MultiArray()
        vel_msg.data = [
            self.vel['x'], self.vel['y'], self.vel['z'],
            math.radians(self.vel['rx']),
            math.radians(self.vel['ry']),
            math.radians(self.vel['rz']),
        ]
        self.zit6_vel_pub.publish(vel_msg)
        self.vel_count += 1

    def _publish_sim_nav(self) -> None:
        """``pos_world[6] + vel_body[6]`` for the MCU's SITL navigation injection."""
        nav = Float32MultiArray()
        nav.data = self._sim_pos + self._sim_vel
        self.sim_nav_pub.publish(nav)
        self.sim_nav_count += 1


def main(args=None) -> int:
    return runner.run(NavSimNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
