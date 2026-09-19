"""Reproducible IMU dropout and bias/noise adapter for SIL experiments."""

from __future__ import annotations

import json
import random
import time

from auv_protocol.topics import (
    IMU,
    SIM_DEGRADATION_EVENTS,
    SIM_DEGRADED_IMU,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def _vector(value):
    """Accept a ROS list parameter or its YAML-string representation."""
    if isinstance(value, str):
        value = value.strip().strip('[]')
        result = tuple(float(item.strip()) for item in value.split(',')
                       if item.strip())
    else:
        result = tuple(float(item) for item in value)
    if len(result) != 3:
        raise ValueError('IMU bias vectors must contain exactly three values')
    return result


def degrade_imu(message: Imu, *, rng: random.Random,
                dropout_probability: float = 0.0,
                accelerometer_noise_stddev: float = 0.0,
                gyroscope_noise_stddev: float = 0.0,
                accelerometer_bias=(0.0, 0.0, 0.0),
                gyroscope_bias=(0.0, 0.0, 0.0)) -> Imu | None:
    """Copy an IMU message and apply deterministic bias and Gaussian noise."""
    if rng.random() < float(dropout_probability):
        return None
    output = Imu()
    output.header = message.header
    output.orientation = message.orientation
    output.orientation_covariance = list(message.orientation_covariance)
    output.linear_acceleration_covariance = list(
        message.linear_acceleration_covariance)
    output.angular_velocity_covariance = list(message.angular_velocity_covariance)
    accel_bias = tuple(float(value) for value in accelerometer_bias)
    gyro_bias = tuple(float(value) for value in gyroscope_bias)
    accel = (message.linear_acceleration.x,
             message.linear_acceleration.y,
             message.linear_acceleration.z)
    gyro = (message.angular_velocity.x,
            message.angular_velocity.y,
            message.angular_velocity.z)
    accel_values = [value + accel_bias[index] + rng.gauss(
        0.0, float(accelerometer_noise_stddev))
                    for index, value in enumerate(accel)]
    gyro_values = [value + gyro_bias[index] + rng.gauss(
        0.0, float(gyroscope_noise_stddev))
                   for index, value in enumerate(gyro)]
    (output.linear_acceleration.x,
     output.linear_acceleration.y,
     output.linear_acceleration.z) = accel_values
    (output.angular_velocity.x,
     output.angular_velocity.y,
     output.angular_velocity.z) = gyro_values
    return output


class ImuDegradationNode(Node):
    """Publish degraded IMU samples on an explicit non-feedback topic."""

    def __init__(self) -> None:
        super().__init__('imu_degradation')
        self.declare_parameter('input_topic', IMU)
        self.declare_parameter('output_topic', SIM_DEGRADED_IMU)
        self.declare_parameter('dropout_probability', 0.0)
        self.declare_parameter('accelerometer_noise_stddev', 0.0)
        self.declare_parameter('gyroscope_noise_stddev', 0.0)
        self.declare_parameter('accelerometer_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('gyroscope_bias', [0.0, 0.0, 0.0])
        self.declare_parameter('random_walk_stddev', 0.0)
        self.declare_parameter('delay_ms', 0.0)
        self.declare_parameter('seed', 0)
        self._rng = random.Random(int(self.get_parameter('seed').value))
        self._dropout = float(self.get_parameter('dropout_probability').value)
        self._accel_noise = float(
            self.get_parameter('accelerometer_noise_stddev').value)
        self._gyro_noise = float(
            self.get_parameter('gyroscope_noise_stddev').value)
        self._accel_bias = _vector(
            self.get_parameter('accelerometer_bias').value)
        self._gyro_bias = _vector(
            self.get_parameter('gyroscope_bias').value)
        self._random_walk_stddev = float(
            self.get_parameter('random_walk_stddev').value)
        self._random_walk = [0.0] * 3
        self._delay_s = max(
            0.0, float(self.get_parameter('delay_ms').value) / 1000.0)
        self._pending = []
        output_topic = str(self.get_parameter('output_topic').value)
        self._pub = self.create_publisher(Imu, output_topic, 10)
        self._events = self.create_publisher(String, SIM_DEGRADATION_EVENTS, 10)
        input_topic = str(self.get_parameter('input_topic').value)
        self.create_subscription(Imu, input_topic, self._callback, 10)
        if self._delay_s > 0.0:
            self.create_timer(0.01, self._flush_pending)

    def _publish_or_delay(self, message: Imu) -> None:
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

    def _callback(self, message: Imu) -> None:
        if self._random_walk_stddev > 0.0:
            self._random_walk = [
                value + self._rng.gauss(0.0, self._random_walk_stddev)
                for value in self._random_walk]
        accel_bias = tuple(
            bias + self._random_walk[index]
            for index, bias in enumerate(self._accel_bias))
        gyro_bias = tuple(
            bias + self._random_walk[index]
            for index, bias in enumerate(self._gyro_bias))
        output = degrade_imu(
            message, rng=self._rng, dropout_probability=self._dropout,
            accelerometer_noise_stddev=self._accel_noise,
            gyroscope_noise_stddev=self._gyro_noise,
            accelerometer_bias=accel_bias,
            gyroscope_bias=gyro_bias)
        if output is None:
            event = String()
            event.data = json.dumps({'event': 'imu_dropout'})
            self._events.publish(event)
            return
        self._publish_or_delay(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuDegradationNode()
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
