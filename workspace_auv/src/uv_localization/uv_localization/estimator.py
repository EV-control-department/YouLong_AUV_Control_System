"""Adapt the MCU's versioned odom into canonical state and TF.

The MCU owns navigation selection and nav-to-odom conversion in every profile.
This boundary neither integrates sensors nor subtracts another origin.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
import threading
import time

from auv_protocol.topics import (
    STATE_HEALTH, STATE_ODOM, STATE_RESET, STATE_RESET_RESULT, STATE_TWIST,
    TF, ZIT6_ODOM, ZIT6_SET_ORIGIN,
)
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from uv_msgs.msg import PoseInfo, SensorHealth, StateResetRequest, StateResetResult
from zit6_interfaces.msg import ZitOdom
from zit6_interfaces.srv import SetOrigin


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _at_least_u32(value: int, reference: int) -> bool:
    """Compare MCU counters, including uint32 rollover."""
    return ((int(value) - int(reference)) & 0xffffffff) < 0x80000000


@dataclass
class _PendingReset:
    request_id: int
    future: object
    started_at: float
    deadline: float
    baseline_generation: int
    boot_epoch: int
    origin_generation: int | None = None
    nav_timestamp_ms: int = 0


class EstimatorNode(Node):
    """Publish exactly the MCU odom and confirm applied reset generations."""

    def __init__(self) -> None:
        super().__init__('uv_localization')
        # Existing launch arguments remain accepted for profile compatibility.
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('estimator', 'bootstrap')
        if str(self.get_parameter('estimator').value).strip().lower() != 'bootstrap':
            raise ValueError('this localization boundary supports only the MCU bootstrap adapter')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('position_timeout', 2.0)
        self.declare_parameter('setorigin_timeout', 3.0)
        self.declare_parameter('reset_frame_timeout', 1.0)
        self._publish_tf_enabled = _as_bool(self.get_parameter('publish_tf').value)
        self._position_timeout = float(self.get_parameter('position_timeout').value)
        self._setorigin_timeout = float(self.get_parameter('setorigin_timeout').value)
        self._reset_frame_timeout = float(self.get_parameter('reset_frame_timeout').value)
        self._lock = threading.Lock()
        self._odom = None
        self._last_odom_time = 0.0
        self._last_nav_progress_time = 0.0
        self._pending_reset: _PendingReset | None = None
        self._reset_results = OrderedDict()
        self._boot_epoch = 0

        self._odom_pub = self.create_publisher(PoseInfo, STATE_ODOM, 10)
        self._twist_pub = self.create_publisher(
            TwistWithCovarianceStamped, STATE_TWIST, 10)
        self._health_pub = self.create_publisher(SensorHealth, STATE_HEALTH, 10)
        self._tf_pub = self.create_publisher(TFMessage, TF, 10)
        self._reset_result_pub = self.create_publisher(
            StateResetResult, STATE_RESET_RESULT, 10)
        self._origin_client = self.create_client(
            SetOrigin, ZIT6_SET_ORIGIN, callback_group=ReentrantCallbackGroup())
        self.create_subscription(ZitOdom, ZIT6_ODOM, self._odom_cb, 10)
        self.create_subscription(
            StateResetRequest, STATE_RESET, self._reset_cb, 10)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._publish_tick)
        self.create_timer(0.05, self._reset_timeout_tick)
        self.get_logger().info('Localization adapting versioned MCU odom')

    @staticmethod
    def _valid_odom(message) -> bool:
        return (len(message.pose_odom) == 6 and len(message.twist_body) == 6
                and _finite(message.pose_odom) and _finite(message.twist_body))

    def _odom_cb(self, message: ZitOdom) -> None:
        if not self._valid_odom(message):
            return
        now = time.monotonic()
        with self._lock:
            previous = self._odom
            restarted = bool(previous is not None and (
                not _at_least_u32(message.nav_timestamp_ms,
                                  previous.nav_timestamp_ms)
                or (previous.origin_initialized and not message.origin_initialized)
                or (previous.origin_initialized and message.origin_initialized
                    and not _at_least_u32(message.origin_generation,
                                          previous.origin_generation))))
            if restarted:
                self._boot_epoch += 1
                self._reset_results.clear()
            if (restarted or previous is None
                    or message.nav_timestamp_ms != previous.nav_timestamp_ms):
                self._last_nav_progress_time = now
            self._odom = message
            self._last_odom_time = now
            pending = self._pending_reset
        if restarted and pending is not None:
            self._finish_reset(pending, False, 'MCU navigation/origin restarted')
            if not pending.future.done():
                self._origin_client.remove_pending_request(pending.future)
                pending.future.cancel()
        elif pending is not None and self._reset_frame_matches(pending):
            self._finish_reset(pending, True, 'MCU odom origin set')

    def _reset_frame_matches(self, pending: _PendingReset) -> bool:
        with self._lock:
            message = self._odom
            return bool(
                self._pending_reset is pending
                and pending.boot_epoch == self._boot_epoch
                and pending.origin_generation is not None
                and message is not None and message.origin_initialized
                and int(message.origin_generation) == pending.origin_generation
                and _at_least_u32(message.nav_timestamp_ms,
                                  pending.nav_timestamp_ms)
                and self._last_odom_time >= pending.started_at
                and time.monotonic() - self._last_odom_time
                <= self._position_timeout
                and time.monotonic() - self._last_nav_progress_time
                <= self._position_timeout)

    def _reset_cb(self, request: StateResetRequest) -> None:
        request_id = int(request.request_id)
        with self._lock:
            cached = self._reset_results.get(request_id)
            pending = self._pending_reset
            baseline = int(self._odom.origin_generation) if self._odom is not None else 0
            boot_epoch = self._boot_epoch
        if cached is not None:
            self._reset_result_pub.publish(cached)
            return
        if pending is not None:
            if pending.request_id != request_id:
                self._publish_reset_result(request_id, False, 'origin reset busy')
            return
        if not self._origin_client.service_is_ready():
            self._publish_reset_result(request_id, False, 'setorigin service unavailable')
            return
        try:
            future = self._origin_client.call_async(SetOrigin.Request())
        except Exception as error:
            self._publish_reset_result(request_id, False, str(error))
            return
        now = time.monotonic()
        pending = _PendingReset(request_id, future, now,
                                now + self._setorigin_timeout, baseline, boot_epoch)
        with self._lock:
            self._pending_reset = pending
        future.add_done_callback(lambda completed: self._origin_done(pending, completed))

    def _origin_done(self, pending: _PendingReset, future) -> None:
        with self._lock:
            if self._pending_reset is not pending:
                return
        try:
            response = future.result()
            if response is None:
                raise RuntimeError('setorigin returned no response')
        except Exception as error:
            self._finish_reset(pending, False, str(error))
            return
        if not response.success:
            self._finish_reset(pending, False, response.message)
            return
        if len(response.origin_nav) != 6 or not _finite(response.origin_nav):
            self._finish_reset(pending, False, 'invalid setorigin response')
            return
        response_generation = int(response.origin_generation)
        advance = (response_generation - pending.baseline_generation) & 0xffffffff
        if response_generation == 0 or not (0 < advance < 0x80000000):
            self._finish_reset(pending, False, 'unexpected setorigin generation')
            return
        self.get_logger().info(
            'MCU adopted nav origin: xyz={} yaw={:.3f} rad generation={}'.format(
                tuple(float(value) for value in response.origin_nav[:3]),
                float(response.origin_nav[5]), int(response.origin_generation)))
        with self._lock:
            if self._pending_reset is not pending:
                return
            pending.origin_generation = int(response.origin_generation)
            pending.nav_timestamp_ms = int(response.nav_timestamp_ms)
            pending.deadline = time.monotonic() + self._reset_frame_timeout
        if self._reset_frame_matches(pending):
            self._finish_reset(pending, True, 'MCU odom origin set')

    def _reset_timeout_tick(self) -> None:
        with self._lock:
            pending = self._pending_reset
        if pending is not None and time.monotonic() >= pending.deadline:
            stage = ('setorigin response timeout' if pending.origin_generation is None
                     else 'matching MCU odom timeout')
            self._finish_reset(pending, False, stage)
            if not pending.future.done():
                self._origin_client.remove_pending_request(pending.future)
                pending.future.cancel()

    def _finish_reset(self, pending: _PendingReset, success: bool, message: str) -> None:
        with self._lock:
            if self._pending_reset is not pending:
                return
            self._pending_reset = None
        if success:
            # Publish confirmed odom before completion; BasicMotion also gates
            # on the PoseInfo generation before emitting an arm heartbeat.
            self._publish_tick()
        self._publish_reset_result(pending.request_id, success, message,
                                   pending.origin_generation)

    def _publish_reset_result(self, request_id, success, message, generation=None) -> None:
        result = StateResetResult()
        result.request_id = int(request_id)
        result.success = bool(success)
        result.message = str(message)
        with self._lock:
            current = self._odom
            result.origin_generation = int(
                generation if generation is not None
                else current.origin_generation if current is not None else 0)
            self._reset_results[int(request_id)] = result
            while len(self._reset_results) > 32:
                self._reset_results.popitem(last=False)
        self._reset_result_pub.publish(result)

    def _publish_tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            message = self._odom
            fresh = (message is not None
                     and now - self._last_odom_time <= self._position_timeout
                     and now - self._last_nav_progress_time <= self._position_timeout)
        stamp = self.get_clock().now().to_msg()
        pose = PoseInfo()
        pose.stamp = stamp
        # origin_* remain zero: every pose and target uses MCU odom directly.
        if message is not None:
            values = message.pose_odom
            pose.robot_x, pose.robot_y, pose.robot_z = map(float, values[:3])
            pose.robot_roll, pose.robot_pitch, pose.robot_yaw = (
                math.degrees(float(value)) for value in values[3:])
            pose.origin_initialized = bool(message.origin_initialized)
            # TEMPORARY override: trust finite, fresh MCU odom even when the
            # MCU's nav_valid flag is false. Freshness is still required.
            pose.nav_valid = bool(fresh)
            pose.origin_generation = int(message.origin_generation)
            pose.nav_timestamp_ms = int(message.nav_timestamp_ms)
        self._odom_pub.publish(pose)

        twist = TwistWithCovarianceStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = 'base_link'
        if message is not None and fresh:
            values = message.twist_body
            twist.twist.twist.linear.x = float(values[0])
            twist.twist.twist.linear.y = float(values[1])
            twist.twist.twist.linear.z = float(values[2])
            twist.twist.twist.angular.x = float(values[3])
            twist.twist.twist.angular.y = float(values[4])
            twist.twist.twist.angular.z = float(values[5])
        self._twist_pub.publish(twist)

        health = SensorHealth()
        health.header.stamp = stamp
        health.header.frame_id = 'base_link'
        health.sensor_name = 'localization'
        health.available = bool(pose.nav_valid)
        health.quality = 1.0 if health.available else 0.0
        health.detail = (
            'fresh MCU odom; nav_valid ignored'
            if health.available and pose.origin_initialized else
            'fresh MCU odom; origin not initialized; nav_valid ignored'
            if health.available else 'MCU odom stale')
        self._health_pub.publish(health)

        if self._publish_tf_enabled and pose.origin_initialized and pose.nav_valid:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = pose.robot_x
            transform.transform.translation.y = pose.robot_y
            transform.transform.translation.z = pose.robot_z
            roll, pitch, yaw = (float(value) / 2.0 for value in message.pose_odom[3:])
            cr, sr = math.cos(roll), math.sin(roll)
            cp, sp = math.cos(pitch), math.sin(pitch)
            cy, sy = math.cos(yaw), math.sin(yaw)
            transform.transform.rotation.x = sr * cp * cy - cr * sp * sy
            transform.transform.rotation.y = cr * sp * cy + sr * cp * sy
            transform.transform.rotation.z = cr * cp * sy - sr * sp * cy
            transform.transform.rotation.w = cr * cp * cy + sr * sp * sy
            self._tf_pub.publish(TFMessage(transforms=[transform]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
