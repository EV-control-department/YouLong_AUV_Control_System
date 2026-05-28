"""Minimal control node: the ONLY node that talks to hardware/sim_bridge.

Provides primitive motion commands:
- set_world(x, y, z, yaw) — world frame absolute position
- set_body(x, y, z, yaw) — body frame absolute position
- set_step(dx, dy, dz, dyaw) — incremental body frame step
- vel_world(vx, vy, vz, vyaw) — world frame velocity
- vel_body(vx, vy, vz, vyaw) — body frame velocity
- vel_step(dvx, dvy, dvz, dvyaw) — incremental body frame velocity
- arm_control(cmd) — arm/disarm
- led_control(cmd) — LED control
"""

import math
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from uv_msgs.msg import MotionCommand, AuvState


class MinimalControlNode(Node):
    """Minimal control node: hardware abstraction boundary."""

    def __init__(self):
        super().__init__('minimal_control')

        # Current state
        self.state = AuvState()
        self._state_lock = threading.Lock()

        # Publisher
        self.pub_motion = self.create_publisher(MotionCommand, '/auv/cmd/motion', 10)

        # Subscriber
        self.create_subscription(AuvState, '/auv/state', self._state_cb, 10)

        # Services
        self.create_service(Trigger, '/auv/cmd/arm', self._arm_cb)

        self.get_logger().info('MinimalControl node started')

    def _state_cb(self, msg: AuvState):
        with self._state_lock:
            self.state = msg

    def get_state(self) -> AuvState:
        with self._state_lock:
            return self.state

    # ========================================================================
    # Primitive command methods
    # ========================================================================

    def set_world(self, x: float, y: float, z: float, yaw: float):
        """Absolute world-frame position command. Yaw in radians."""
        self._publish_motion(MotionCommand.MODE_POS_WORLD, 0, x, y, z, yaw)

    def set_body(self, x: float, y: float, z: float, yaw: float):
        """Absolute body-frame position command. Yaw in radians."""
        self._publish_motion(MotionCommand.MODE_POS_BODY, 0, x, y, z, yaw)

    def set_step(self, dx: float, dy: float, dz: float, dyaw: float):
        """Incremental body-frame position step. Yaw in radians."""
        self._publish_motion(MotionCommand.MODE_POS_BODY_STEP, 0, dx, dy, dz, dyaw)

    def vel_world(self, vx: float, vy: float, vz: float, vyaw: float):
        """World-frame velocity command. vyaw in rad/s."""
        self._publish_motion(MotionCommand.MODE_VEL_WORLD, 0, vx, vy, vz, vyaw)

    def vel_body(self, vx: float, vy: float, vz: float, vyaw: float):
        """Body-frame velocity command. vyaw in rad/s."""
        self._publish_motion(MotionCommand.MODE_VEL_BODY, 0, vx, vy, vz, vyaw)

    def vel_step(self, dvx: float, dvy: float, dvz: float, dvyaw: float):
        """Incremental body-frame velocity step."""
        self._publish_motion(MotionCommand.MODE_VEL_BODY_STEP, 0, dvx, dvy, dvz, dvyaw)

    def arm_control(self, arm_cmd: int):
        """Send arm command."""
        self.get_logger().info(f'Arm command: {arm_cmd}')

    def led_control(self, led_cmd: int):
        """Send LED command."""
        self.get_logger().info(f'LED command: {led_cmd}')

    # ========================================================================
    # Move wait
    # ========================================================================

    def move_wait(self, axes_mask: int = 0x0F, timeout: float = 30.0,
                  tol_x: float = 0.1, tol_y: float = 0.1,
                  tol_z: float = 0.1, tol_rz: float = 5.0) -> bool:
        """Block until current position reaches target or timeout.

        Args:
            axes_mask: Bitmask of axes to check (bit0=x, bit1=y, bit2=z, bit3=yaw)
            timeout: Maximum wait time in seconds
            tol_x, tol_y, tol_z: Position tolerance in meters
            tol_rz: Yaw tolerance in degrees

        Returns:
            True if reached, False if timeout
        """
        rate = self.create_rate(10)
        start = self.get_clock().now()

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                self.get_logger().warn('move_wait timeout')
                return False

            s = self.get_state()
            reached = True

            if axes_mask & 0x01:
                if abs(s.target_x - s.pos_x) > tol_x:
                    reached = False
            if axes_mask & 0x02:
                if abs(s.target_y - s.pos_y) > tol_y:
                    reached = False
            if axes_mask & 0x04:
                if abs(s.target_z - s.pos_z) > tol_z:
                    reached = False
            if axes_mask & 0x08:
                yaw_err = abs(math.degrees(s.target_yaw) - math.degrees(s.yaw))
                if yaw_err > 180:
                    yaw_err = 360 - yaw_err
                if yaw_err > tol_rz:
                    reached = False

            if reached:
                return True

            rate.sleep()

        return False

    # ========================================================================
    # Internal
    # ========================================================================

    def _publish_motion(self, mode: int, type_mask: int,
                        x: float, y: float, z: float, yaw: float):
        msg = MotionCommand()
        msg.mode = mode
        msg.type_mask = type_mask
        msg.x = x
        msg.y = y
        msg.z = z
        msg.yaw = yaw
        self.pub_motion.publish(msg)

    def _arm_cb(self, request, response):
        response.success = True
        response.message = 'Arm command received'
        return response


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
