"""DVL degradation adapter with reproducible dropout and Gaussian noise."""

from __future__ import annotations

import json
import random
import time

from auv_protocol.topics import (
    DVL_ALTITUDE,
    DVL_VELOCITY,
    SIM_DEGRADATION_EVENTS,
    SIM_DEGRADED_DVL_ALTITUDE,
    SIM_DEGRADED_DVL_VELOCITY,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from uv_msgs.msg import DvlAltitude, DvlVelocity


def degrade_velocity(velocity, *, rng, dropout_probability, noise_stddev):
    """Return a copied velocity tuple or ``None`` for a dropped sample."""
    if rng.random() < float(dropout_probability):
        return None
    return tuple(float(value) + rng.gauss(0.0, float(noise_stddev))
                 for value in velocity)


def degrade_altitude(message: DvlAltitude, *, rng: random.Random,
                     dropout_probability: float = 0.0,
                     noise_stddev: float = 0.0,
                     bottom_lock_loss_probability: float = 0.0
                     ) -> DvlAltitude | None:
    """Copy altitude and model dropout, noise and bottom-lock loss."""
    if rng.random() < float(dropout_probability):
        return None
    output = DvlAltitude()
    output.header = message.header
    output.altitude = float(message.altitude) + rng.gauss(
        0.0, float(noise_stddev))
    output.valid = bool(message.valid)
    if (output.valid and rng.random()
            < float(bottom_lock_loss_probability)):
        output.valid = False
    return output


class DvlDegradationNode(Node):
    """Publish degraded DVL on an explicit output topic.

    The default output is under ``/auv/sim/degraded`` so enabling this node
    cannot accidentally create a same-topic feedback loop.  Experiments may
    remap the output to the canonical input after disabling the raw adapter.
    """

    def __init__(self) -> None:
        super().__init__('uv_sim_degradation')
        self.declare_parameter('input_topic', DVL_VELOCITY)
        self.declare_parameter('output_topic', SIM_DEGRADED_DVL_VELOCITY)
        self.declare_parameter('altitude_input_topic', DVL_ALTITUDE)
        self.declare_parameter(
            'altitude_output_topic', SIM_DEGRADED_DVL_ALTITUDE)
        self.declare_parameter('dropout_probability', 0.0)
        self.declare_parameter('beam_loss_probability', 0.0)
        self.declare_parameter('noise_stddev', 0.0)
        self.declare_parameter('altitude_noise_stddev', 0.0)
        self.declare_parameter('bottom_lock_loss_probability', 0.0)
        self.declare_parameter('delay_ms', 0.0)
        self.declare_parameter('seed', 0)
        self._rng = random.Random(int(self.get_parameter('seed').value))
        self._dropout_probability = float(
            self.get_parameter('dropout_probability').value)
        self._beam_loss_probability = float(
            self.get_parameter('beam_loss_probability').value)
        self._noise_stddev = float(self.get_parameter('noise_stddev').value)
        self._altitude_noise_stddev = float(
            self.get_parameter('altitude_noise_stddev').value)
        self._bottom_lock_loss_probability = float(
            self.get_parameter('bottom_lock_loss_probability').value)
        self._delay_s = max(
            0.0, float(self.get_parameter('delay_ms').value) / 1000.0)
        self._pending = []
        output_topic = str(self.get_parameter('output_topic').value)
        input_topic = str(self.get_parameter('input_topic').value)
        self._pub = self.create_publisher(DvlVelocity, output_topic, 10)
        altitude_output_topic = str(
            self.get_parameter('altitude_output_topic').value)
        self._altitude_pub = self.create_publisher(
            DvlAltitude, altitude_output_topic, 10)
        self._events = self.create_publisher(String, SIM_DEGRADATION_EVENTS, 10)
        self.create_subscription(DvlVelocity, input_topic, self._callback, 10)
        altitude_input_topic = str(
            self.get_parameter('altitude_input_topic').value)
        self.create_subscription(
            DvlAltitude, altitude_input_topic, self._altitude_callback, 10)
        if self._delay_s > 0.0:
            self.create_timer(0.01, self._flush_pending)
        self.get_logger().info(
            f'DVL degradation: {input_topic} -> {output_topic}, '
            f'dropout={self._dropout_probability:.3f}, '
            f'noise={self._noise_stddev:.3f}, '
            f'delay_ms={self._delay_s * 1000.0:.1f}, '
            f'seed={self.get_parameter("seed").value}')

    def _publish_or_delay(self, message: DvlVelocity) -> None:
        if self._delay_s <= 0.0:
            self._pub.publish(message)
            return
        self._pending.append((time.monotonic() + self._delay_s, message))

    def _flush_pending(self) -> None:
        now = time.monotonic()
        ready = []
        while self._pending and self._pending[0][0] <= now:
            ready.append(self._pending.pop(0)[1])
        for message in ready:
            self._pub.publish(message)

    def _callback(self, msg: DvlVelocity) -> None:
        values = (msg.velocity.x, msg.velocity.y, msg.velocity.z)
        degraded = degrade_velocity(
            values, rng=self._rng,
            dropout_probability=self._dropout_probability,
            noise_stddev=self._noise_stddev)
        if degraded is None:
            event = String()
            event.data = json.dumps({'event': 'dvl_dropout'})
            self._events.publish(event)
            return
        output = DvlVelocity()
        output.header = msg.header
        output.velocity = msg.velocity
        output.velocity.x, output.velocity.y, output.velocity.z = degraded
        output.velocity_covariance = list(msg.velocity_covariance)
        output.valid = bool(msg.valid)
        if (output.valid and self._rng.random()
                < self._beam_loss_probability):
            output.valid = False
            event = String()
            event.data = json.dumps({'event': 'dvl_beam_loss'})
            self._events.publish(event)
        self._publish_or_delay(output)

    def _altitude_callback(self, msg: DvlAltitude) -> None:
        output = degrade_altitude(
            msg, rng=self._rng,
            dropout_probability=self._dropout_probability,
            noise_stddev=self._altitude_noise_stddev,
            bottom_lock_loss_probability=self._bottom_lock_loss_probability)
        if output is None:
            event = String()
            event.data = json.dumps({'event': 'dvl_bottom_lock_dropout'})
            self._events.publish(event)
            return
        self._altitude_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DvlDegradationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
