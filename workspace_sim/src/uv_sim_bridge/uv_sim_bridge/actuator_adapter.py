"""Canonical force command to Stonefish thruster command adapter."""

from __future__ import annotations

import rclpy

from auv_protocol.topics import SIM_THRUSTER_COMMAND
from std_msgs.msg import Float64MultiArray


class ActuatorAdapter:
    """Keep simulator actuation transport independent from ZIT6 emulation."""

    def __init__(self, node, mixer):
        self._node = node
        self._mixer = mixer
        self._publisher = node.create_publisher(
            Float64MultiArray, SIM_THRUSTER_COMMAND, 10)

    def publish_force(self, force_6dof) -> list[float]:
        thrust = list(self._mixer.mix6(*force_6dof))
        self.publish_thrust(thrust)
        return thrust

    def publish_thrust(self, thrust) -> None:
        """Publish already-mixed thruster values to the simulator."""
        if getattr(self._node, '_shutdown_requested', False) or not rclpy.ok():
            return
        message = Float64MultiArray()
        message.data = list(thrust)
        try:
            self._publisher.publish(message)
        except Exception:
            # SIGINT can invalidate DDS handles between the check above and
            # publish().  Suppress only that expected shutdown race.
            if getattr(self._node, '_shutdown_requested', False) or not rclpy.ok():
                return
            raise
