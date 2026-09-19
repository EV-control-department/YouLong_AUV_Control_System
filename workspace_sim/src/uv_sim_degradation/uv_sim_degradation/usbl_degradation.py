"""USBL dropout and position-noise adapter for SIL experiments."""

from __future__ import annotations

import json
import random
import time

from auv_protocol.topics import (
    SIM_DEGRADATION_EVENTS,
    SIM_DEGRADED_USBL,
    USBL_MEASUREMENT,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from uv_msgs.msg import UsblMeasurement


def degrade_usbl(message: UsblMeasurement, *, rng: random.Random,
                 dropout_probability: float = 0.0,
                 position_noise_stddev: float = 0.0,
                 outlier_probability: float = 0.0,
                 outlier_stddev: float = 0.0) -> UsblMeasurement | None:
    """Copy a USBL measurement and add isotropic position noise."""
    if rng.random() < float(dropout_probability):
        return None
    output = UsblMeasurement()
    output.header = message.header
    output.position = message.position
    output.position_covariance = list(message.position_covariance)
    output.valid = bool(message.valid)
    noise = float(position_noise_stddev)
    if rng.random() < float(outlier_probability):
        # Keep the sample marked valid: an outlier is deliberately different
        # from a dropout so estimator gating can be evaluated.
        noise = max(noise, float(outlier_stddev))
    output.position.x += rng.gauss(0.0, noise)
    output.position.y += rng.gauss(0.0, noise)
    output.position.z += rng.gauss(0.0, noise)
    return output


class UsblDegradationNode(Node):
    """Publish degraded USBL on an explicit non-feedback topic."""

    def __init__(self) -> None:
        super().__init__('usbl_degradation')
        self.declare_parameter('input_topic', USBL_MEASUREMENT)
        self.declare_parameter('output_topic', SIM_DEGRADED_USBL)
        self.declare_parameter('dropout_probability', 0.0)
        self.declare_parameter('position_noise_stddev', 0.0)
        self.declare_parameter('outlier_probability', 0.0)
        self.declare_parameter('outlier_stddev', 0.0)
        self.declare_parameter('delay_ms', 0.0)
        self.declare_parameter('seed', 0)
        self._rng = random.Random(int(self.get_parameter('seed').value))
        self._dropout = float(self.get_parameter('dropout_probability').value)
        self._noise = float(
            self.get_parameter('position_noise_stddev').value)
        self._outlier_probability = float(
            self.get_parameter('outlier_probability').value)
        self._outlier_stddev = float(
            self.get_parameter('outlier_stddev').value)
        self._delay_s = max(
            0.0, float(self.get_parameter('delay_ms').value) / 1000.0)
        self._pending = []
        output_topic = str(self.get_parameter('output_topic').value)
        input_topic = str(self.get_parameter('input_topic').value)
        self._pub = self.create_publisher(UsblMeasurement, output_topic, 10)
        self._events = self.create_publisher(String, SIM_DEGRADATION_EVENTS, 10)
        self.create_subscription(UsblMeasurement, input_topic, self._callback, 10)
        if self._delay_s > 0.0:
            self.create_timer(0.01, self._flush_pending)

    def _publish_or_delay(self, message: UsblMeasurement) -> None:
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

    def _callback(self, message: UsblMeasurement) -> None:
        output = degrade_usbl(
            message, rng=self._rng, dropout_probability=self._dropout,
            position_noise_stddev=self._noise,
            outlier_probability=self._outlier_probability,
            outlier_stddev=self._outlier_stddev)
        if output is None:
            event = String()
            event.data = json.dumps({'event': 'usbl_dropout'})
            self._events.publish(event)
            return
        self._publish_or_delay(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UsblDegradationNode()
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
