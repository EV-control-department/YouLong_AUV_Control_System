"""Basic motion node: high-level motion API.

Does NOT talk to hardware directly. Publishes MotionCommand to /auv/cmd/motion.
Implements: SET functions (absolute closed-loop), WMOVE functions (world-frame stepping),
BMOVE functions (body-frame stepping).

Functions form a matrix of axis combinations × motion types, and call each other internally.
"""

import math
import threading

import rclpy
from rclpy.node import Node

from uv_msgs.msg import MotionCommand, AuvState
from uv_control.coordinate_utils import (
    wrap_angle_rad, wrap_angle_deg,
    world_to_body, body_to_world,
)

# Axis masks
AX_X = 0x01
AX_Y = 0x02
AX_Z = 0x04
AX_RZ = 0x08
AX_XY = AX_X | AX_Y
AX_XYZ = AX_X | AX_Y | AX_Z
AX_ALL = AX_X | AX_Y | AX_Z | AX_RZ

# Default tolerances
TOL_X = 0.1    # meters
TOL_Y = 0.1    # meters
TOL_Z = 0.1    # meters
TOL_RZ = 5.0   # degrees


class BasicMotionNode(Node):
    """High-level motion API. Does NOT talk to hardware directly."""

    def __init__(self):
        super().__init__('basic_motion')

        self.state = AuvState()
        self._state_lock = threading.Lock()

        self.pub_motion = self.create_publisher(MotionCommand, '/auv/cmd/motion', 10)
        self.create_subscription(AuvState, '/auv/state', self._state_cb, 10)

        self.get_logger().info('BasicMotion node started')

    def _state_cb(self, msg: AuvState):
        with self._state_lock:
            self.state = msg

    def get_state(self) -> AuvState:
        with self._state_lock:
            return self.state

    # ========================================================================
    # Internal: send command and wait
    # ========================================================================

    def _cmd_and_wait(self, mode: int, x: float, y: float, z: float, yaw: float,
                      axes_mask: int = AX_ALL,
                      tol_x: float = TOL_X, tol_y: float = TOL_Y,
                      tol_z: float = TOL_Z, tol_rz: float = TOL_RZ,
                      timeout: float = 60.0) -> bool:
        """Send motion command and wait until position reached."""
        msg = MotionCommand()
        msg.mode = mode
        msg.type_mask = 0
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = yaw
        self.pub_motion.publish(msg)

        return self._wait_reached(axes_mask, tol_x, tol_y, tol_z, tol_rz, timeout)

    def _wait_reached(self, axes_mask: int = AX_ALL,
                      tol_x: float = TOL_X, tol_y: float = TOL_Y,
                      tol_z: float = TOL_Z, tol_rz: float = TOL_RZ,
                      timeout: float = 60.0) -> bool:
        """Block until current position reaches target or timeout."""
        rate = self.create_rate(10)
        start = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn('move_wait timeout')
                return False

            s = self.get_state()
            reached = True

            if axes_mask & AX_X:
                if abs(s.target_x - s.pos_x) > tol_x:
                    reached = False
            if axes_mask & AX_Y:
                if abs(s.target_y - s.pos_y) > tol_y:
                    reached = False
            if axes_mask & AX_Z:
                if abs(s.target_z - s.pos_z) > tol_z:
                    reached = False
            if axes_mask & AX_RZ:
                yaw_err = abs(math.degrees(s.target_yaw - s.yaw))
                if yaw_err > 180:
                    yaw_err = 360 - yaw_err
                if yaw_err > tol_rz:
                    reached = False

            if reached:
                return True

            rate.sleep()

        return False

    # ========================================================================
    # SET functions (absolute, closed-loop, world frame)
    # ========================================================================

    def setxyzrz(self, x: float, y: float, z: float, rz: float,
                 timeout: float = 60.0) -> bool:
        """Set absolute world position + yaw. Decomposes into setxy + setz + setrz."""
        rz_rad = math.radians(rz)
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, x, y, z, rz_rad, AX_ALL, timeout=timeout)
        return True

    def setxyz(self, x: float, y: float, z: float, timeout: float = 60.0) -> bool:
        """Set absolute world position (no yaw change)."""
        s = self.get_state()
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, x, y, z, s.yaw, AX_XYZ, timeout=timeout)
        return True

    def setxy(self, x: float, y: float, timeout: float = 60.0) -> bool:
        """Set absolute world XY position."""
        s = self.get_state()
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, x, y, s.pos_z, s.yaw, AX_XY, timeout=timeout)
        return True

    def setxyrz(self, x: float, y: float, rz: float, timeout: float = 60.0) -> bool:
        """Set absolute world XY position + yaw."""
        rz_rad = math.radians(rz)
        s = self.get_state()
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, x, y, s.pos_z, rz_rad,
                           AX_X | AX_Y | AX_RZ, timeout=timeout)
        return True

    def setz(self, z: float, timeout: float = 60.0) -> bool:
        """Set absolute depth."""
        s = self.get_state()
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, s.pos_x, s.pos_y, z, s.yaw,
                           AX_Z, timeout=timeout)
        return True

    def setrz(self, rz: float, timeout: float = 60.0) -> bool:
        """Set absolute yaw (degrees)."""
        rz_rad = math.radians(rz)
        s = self.get_state()
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, s.pos_x, s.pos_y, s.pos_z, rz_rad,
                           AX_RZ, timeout=timeout)
        return True

    def setx(self, x: float, timeout: float = 60.0) -> bool:
        """Set absolute X (North) position."""
        s = self.get_state()
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, x, s.pos_y, s.pos_z, s.yaw,
                           AX_X, timeout=timeout)
        return True

    def sety(self, y: float, timeout: float = 60.0) -> bool:
        """Set absolute Y (East) position."""
        s = self.get_state()
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, s.pos_x, y, s.pos_z, s.yaw,
                           AX_Y, timeout=timeout)
        return True

    # ========================================================================
    # WMOVE functions (world-frame stepping)
    # ========================================================================

    def wmovexyzrz(self, dx: float, dy: float, dz: float, drz: float,
                   timeout: float = 60.0) -> bool:
        """World-frame step: move by (dx, dy, dz, drz) in world coordinates."""
        s = self.get_state()
        target_x = s.pos_x + dx
        target_y = s.pos_y + dy
        target_z = s.pos_z + dz
        target_rz = s.yaw + math.radians(drz)
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, target_y, target_z, target_rz,
                           AX_ALL, timeout=timeout)
        return True

    def wmovexyz(self, dx: float, dy: float, dz: float, timeout: float = 60.0) -> bool:
        """World-frame step: move by (dx, dy, dz)."""
        s = self.get_state()
        target_x = s.pos_x + dx
        target_y = s.pos_y + dy
        target_z = s.pos_z + dz
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, target_y, target_z, s.yaw,
                           AX_XYZ, timeout=timeout)
        return True

    def wmovexy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        """World-frame step: move by (dx, dy)."""
        s = self.get_state()
        target_x = s.pos_x + dx
        target_y = s.pos_y + dy
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, target_y, s.pos_z, s.yaw,
                           AX_XY, timeout=timeout)
        return True

    def wmovexyrz(self, dx: float, dy: float, drz: float, timeout: float = 60.0) -> bool:
        """World-frame step: move by (dx, dy, drz)."""
        s = self.get_state()
        target_x = s.pos_x + dx
        target_y = s.pos_y + dy
        target_rz = s.yaw + math.radians(drz)
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, target_y, s.pos_z, target_rz,
                           AX_X | AX_Y | AX_RZ, timeout=timeout)
        return True

    def wmovez(self, dz: float, timeout: float = 60.0) -> bool:
        """World-frame step: change depth by dz."""
        s = self.get_state()
        target_z = s.pos_z + dz
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, s.pos_x, s.pos_y, target_z, s.yaw,
                           AX_Z, timeout=timeout)
        return True

    def wmoverz(self, drz: float, timeout: float = 60.0) -> bool:
        """World-frame step: rotate by drz degrees."""
        s = self.get_state()
        target_rz = s.yaw + math.radians(drz)
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, s.pos_x, s.pos_y, s.pos_z, target_rz,
                           AX_RZ, timeout=timeout)
        return True

    def wmovex(self, dx: float, timeout: float = 60.0) -> bool:
        """World-frame step: move by dx in North direction."""
        s = self.get_state()
        target_x = s.pos_x + dx
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, s.pos_y, s.pos_z, s.yaw,
                           AX_X, timeout=timeout)
        return True

    def wmovey(self, dy: float, timeout: float = 60.0) -> bool:
        """World-frame step: move by dy in East direction."""
        s = self.get_state()
        target_y = s.pos_y + dy
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, s.pos_x, target_y, s.pos_z, s.yaw,
                           AX_Y, timeout=timeout)
        return True

    # ========================================================================
    # BMOVE functions (body-frame stepping)
    # ========================================================================

    def bmovexyzrz(self, dx: float, dy: float, dz: float, drz: float,
                   timeout: float = 60.0) -> bool:
        """Body-frame step: move by (dx, dy, dz, drz) in body coordinates."""
        s = self.get_state()
        yaw_rad = s.yaw
        dx_w, dy_w = body_to_world(dx, dy, yaw_rad)
        target_x = s.pos_x + dx_w
        target_y = s.pos_y + dy_w
        target_z = s.pos_z + dz
        target_rz = s.yaw + math.radians(drz)
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, target_y, target_z, target_rz,
                           AX_ALL, timeout=timeout)
        return True

    def bmovexyz(self, dx: float, dy: float, dz: float, timeout: float = 60.0) -> bool:
        """Body-frame step: move by (dx, dy, dz)."""
        s = self.get_state()
        dx_w, dy_w = body_to_world(dx, dy, s.yaw)
        target_x = s.pos_x + dx_w
        target_y = s.pos_y + dy_w
        target_z = s.pos_z + dz
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, target_y, target_z, s.yaw,
                           AX_XYZ, timeout=timeout)
        return True

    def bmovexy(self, dx: float, dy: float, timeout: float = 60.0) -> bool:
        """Body-frame step: move by (dx, dy)."""
        s = self.get_state()
        dx_w, dy_w = body_to_world(dx, dy, s.yaw)
        target_x = s.pos_x + dx_w
        target_y = s.pos_y + dy_w
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, target_y, s.pos_z, s.yaw,
                           AX_XY, timeout=timeout)
        return True

    def bmovexyrz(self, dx: float, dy: float, drz: float, timeout: float = 60.0) -> bool:
        """Body-frame step: move by (dx, dy, drz)."""
        s = self.get_state()
        dx_w, dy_w = body_to_world(dx, dy, s.yaw)
        target_x = s.pos_x + dx_w
        target_y = s.pos_y + dy_w
        target_rz = s.yaw + math.radians(drz)
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, target_x, target_y, s.pos_z, target_rz,
                           AX_X | AX_Y | AX_RZ, timeout=timeout)
        return True

    def bmovez(self, dz: float, timeout: float = 60.0) -> bool:
        """Body-frame step: change depth by dz."""
        s = self.get_state()
        target_z = s.pos_z + dz
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, s.pos_x, s.pos_y, target_z, s.yaw,
                           AX_Z, timeout=timeout)
        return True

    def bmoverz(self, drz: float, timeout: float = 60.0) -> bool:
        """Body-frame step: rotate by drz degrees."""
        s = self.get_state()
        target_rz = s.yaw + math.radians(drz)
        self._cmd_and_wait(MotionCommand.MODE_POS_WORLD, s.pos_x, s.pos_y, s.pos_z, target_rz,
                           AX_RZ, timeout=timeout)
        return True

    def bmovex(self, dx: float, timeout: float = 60.0) -> bool:
        """Body-frame step: move forward by dx."""
        return self.bmovexy(dx, 0.0, timeout=timeout)

    def bmovey(self, dy: float, timeout: float = 60.0) -> bool:
        """Body-frame step: move right by dy."""
        return self.bmovexy(0.0, dy, timeout=timeout)


def main(args=None):
    rclpy.init(args=args)
    node = BasicMotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
