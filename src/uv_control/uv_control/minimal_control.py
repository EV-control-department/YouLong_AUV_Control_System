"""Minimal control node: the ONLY node that talks to hardware/sim_bridge.

Communicates via ZIT6 protocol (same as real AUV MCU):
  - Publishes ZitSetpoint to /zit6/cmd/setpoint
  - Subscribes to /zit6/state/status, /zit6/state/pos

Motion primitives:
- set_world(x, y, z, yaw_deg) — world frame absolute position
- set_body(x, y, z, yaw_deg) — body frame absolute position
- set_step(dx, dy, dz, dyaw_deg) — incremental body frame step
- vel_world(vx, vy, vz, vyaw_rad_s) — world frame velocity
- vel_body(vx, vy, vz, vyaw_rad_s) — body frame velocity
- vel_step(dvx, dvy, dvz, dvyaw_rad_s) — incremental body frame velocity
"""

import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from zit6_interfaces.msg import ZitSetpoint, ZitStatus


class MinimalControlNode(Node):
    """Minimal control node: hardware abstraction boundary (ZIT6 protocol)."""

    # control_key constants
    CK_POS = 0       # position mode
    CK_VEL = 1       # velocity mode
    CK_FORCE = 2     # force mode
    CK_BODY = 0x10   # body frame
    CK_INC = 0x20    # incremental

    def __init__(self):
        super().__init__('minimal_control')

        self.state = ZitStatus()
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}  # rz in degrees
        self._lock = threading.Lock()

        # Publisher: ZIT6 setpoint
        self.pub_setpoint = self.create_publisher(ZitSetpoint, '/zit6/cmd/setpoint', 10)

        # Subscribers: ZIT6 state
        self.create_subscription(ZitStatus, '/zit6/state/status', self._status_cb, 10)
        self.create_subscription(Float32MultiArray, '/zit6/state/pos', self._pos_cb, 10)

        self.get_logger().info('MinimalControl node started (ZIT6 protocol)')

    def _status_cb(self, msg: ZitStatus):
        with self._lock:
            self.state = msg

    def _pos_cb(self, msg: Float32MultiArray):
        with self._lock:
            if len(msg.data) >= 4:
                self.pos['x'] = msg.data[0]
                self.pos['y'] = msg.data[1]
                self.pos['z'] = msg.data[2]
                self.pos['rz'] = math.degrees(msg.data[3])

    def get_state(self):
        with self._lock:
            return self.state, self.pos.copy()

    # ========================================================================
    # Primitive commands
    # ========================================================================

    def set_world(self, x: float, y: float, z: float, yaw_deg: float):
        """Absolute world-frame position. yaw in degrees."""
        self._pub(0, 0x0F, x, y, z, yaw_deg)

    def set_body(self, x: float, y: float, z: float, yaw_deg: float):
        """Absolute body-frame position. yaw in degrees."""
        self._pub(self.CK_BODY, 0x0F, x, y, z, yaw_deg)

    def set_step(self, dx: float, dy: float, dz: float, dyaw_deg: float):
        """Incremental body-frame step."""
        self._pub(self.CK_BODY | self.CK_INC, 0x0F, dx, dy, dz, dyaw_deg)

    def vel_world(self, vx: float, vy: float, vz: float, vyaw_rad_s: float):
        """World-frame velocity. vyaw in rad/s."""
        self._pub(self.CK_VEL, 0x0F, vx, vy, vz, math.degrees(vyaw_rad_s))

    def vel_body(self, vx: float, vy: float, vz: float, vyaw_rad_s: float):
        """Body-frame velocity. vyaw in rad/s."""
        self._pub(self.CK_VEL | self.CK_BODY, 0x0F, vx, vy, vz, math.degrees(vyaw_rad_s))

    def vel_step(self, dvx: float, dvy: float, dvz: float, dvyaw_rad_s: float):
        """Incremental body-frame velocity step."""
        self._pub(self.CK_VEL | self.CK_BODY | self.CK_INC, 0x0F, dvx, dvy, dvz, math.degrees(dvyaw_rad_s))

    def set_force(self, fx: float, fy: float, fz: float, mz: float):
        """Direct force/torque command."""
        self._pub(self.CK_FORCE, 0x0F, fx, fy, fz, mz)

    # ========================================================================
    # Move wait
    # ========================================================================

    def move_wait(self, axes_mask: int = 0x0F, timeout: float = 30.0,
                  tol_x: float = 0.1, tol_y: float = 0.1,
                  tol_z: float = 0.1, tol_rz: float = 5.0) -> bool:
        """Block until current position reaches target or timeout.

        Returns True if reached, False if timeout.
        """
        rate = self.create_rate(10)
        start = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn('move_wait timeout')
                return False

            _, pos = self.get_state()
            reached = True

            if axes_mask & 0x01:
                if abs(self._target['x'] - pos['x']) > tol_x:
                    reached = False
            if axes_mask & 0x02:
                if abs(self._target['y'] - pos['y']) > tol_y:
                    reached = False
            if axes_mask & 0x04:
                if abs(self._target['z'] - pos['z']) > tol_z:
                    reached = False
            if axes_mask & 0x08:
                err = abs(self._target['rz'] - pos['rz'])
                if err > 180:
                    err = 360 - err
                if err > tol_rz:
                    reached = False

            if reached:
                return True

            rate.sleep()

        return False

    # ========================================================================
    # Internal
    # ========================================================================

    def _pub(self, control_key: int, type_mask: int,
             x: float, y: float, z: float, yaw_deg: float):
        msg = ZitSetpoint()
        msg.control_key = control_key
        msg.type_mask = type_mask
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = math.radians(yaw_deg)
        msg.seq = 0
        self.pub_setpoint.publish(msg)

        # Track target for move_wait
        mode = control_key & 0x03
        is_body = bool(control_key & self.CK_BODY)
        is_inc = bool(control_key & self.CK_INC)

        with self._lock:
            if mode == 0 and not is_inc:
                # Absolute position target
                if is_body:
                    yaw_rad = math.radians(self.pos['rz'])
                    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                    self._target['x'] = self.pos['x'] + cy * x - sy * y
                    self._target['y'] = self.pos['y'] + sy * x + cy * y
                else:
                    self._target['x'] = x
                    self._target['y'] = y
                self._target['z'] = z
                self._target['rz'] = yaw_deg


def main(args=None):
    rclpy.init(args=args)
    node = MinimalControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
