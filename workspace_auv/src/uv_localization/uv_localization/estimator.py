"""Small canonical-state adapter used as the first localization boundary.

This node intentionally does not subscribe to simulator ground truth.  In a
real deployment it consumes the ZIT6 navigation feedback.  In SIL/HIL it
consumes the canonical DVL and IMU streams and integrates a deliberately
simple dead-reckoning state.  The estimator is a replaceable boundary: the
future FGO backend can publish the same ``/auv/state/*`` interfaces without
changing control, planning, or mission code.

``PoseInfo`` is retained on ``/auv/state/odom`` for compatibility with the
current consumers.  Its yaw fields are degrees; the estimator keeps all
internal angles in radians and converts only at this temporary message edge.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading

from auv_protocol.topics import (
    DVL_VELOCITY,
    IMU,
    LEGACY_ZIT6_POSITION,
    LEGACY_ZIT6_VELOCITY,
    STATE_HEALTH,
    STATE_ODOM,
    STATE_RESET,
    STATE_TWIST,
    TF,
    USBL_MEASUREMENT,
    ZIT6_POSITION,
    ZIT6_VELOCITY,
)
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float32MultiArray
from tf2_msgs.msg import TFMessage
from uv_msgs.msg import DvlVelocity, PoseInfo, SensorHealth, UsblMeasurement


# The simulated Stonefish DVL is mounted away from the vehicle reference
# point. Its message is converted to body axes by uv_sim_bridge, but still
# measures velocity at this sensor origin. Apply the rigid-body lever-arm
# correction only in SIL/HIL where this pose is defined. The real vehicle
# DVL pose is intentionally not guessed from the simulation model.
_SIM_DVL_POSITION_BODY = (-0.375, 0.0, 0.2)


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _wrap_rad(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _as_bool(value) -> bool:
    """Parse launch/YAML booleans without treating ``'false'`` as true."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _body_to_world(vx: float, vy: float, yaw: float) -> tuple[float, float]:
    """FRD body velocity to NED odom velocity (yaw-only first adapter)."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    return cy * vx - sy * vy, sy * vx + cy * vy


def _remove_sensor_lever_arm(
        velocity: tuple[float, float, float],
        angular_velocity: tuple[float, float, float],
        sensor_position: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Remove ``omega × r`` from a sensor velocity in body axes."""
    vx, vy, vz = velocity
    wx, wy, wz = angular_velocity
    rx, ry, rz = sensor_position
    return (
        vx - (wy * rz - wz * ry),
        vy - (wz * rx - wx * rz),
        vz - (wx * ry - wy * rx),
    )


@dataclass
class _State:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    wx: float = 0.0
    wy: float = 0.0
    wz: float = 0.0


class EstimatorNode(Node):
    """Publish one estimated state stream and one dynamic odom TF stream."""

    def __init__(self) -> None:
        super().__init__('uv_localization')
        self.declare_parameter('sim_mode', False)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('position_timeout', 2.0)
        self.declare_parameter('estimator', 'bootstrap')

        self._sim_mode = _as_bool(self.get_parameter('sim_mode').value)
        self._estimator = str(self.get_parameter('estimator').value).strip().lower()
        if self._estimator != 'bootstrap':
            raise ValueError(
                f'unsupported estimator {self._estimator!r}; '
                "V1 supports only 'bootstrap'")
        self._publish_tf_enabled = _as_bool(
            self.get_parameter('publish_tf').value)
        self._position_timeout = float(self.get_parameter('position_timeout').value)
        self._lock = threading.Lock()
        self._state = _State()
        self._raw_position: _State | None = None
        self._map_origin: _State | None = None
        self._last_position_time = None
        self._last_velocity_time = None
        self._last_imu_time = None
        self._last_update_time = self.get_clock().now()
        self._have_measurement = False
        self._sim_sensor_stale_reported = False

        self._odom_pub = self.create_publisher(PoseInfo, STATE_ODOM, 10)
        self._twist_pub = self.create_publisher(
            TwistWithCovarianceStamped, STATE_TWIST, 10)
        self._health_pub = self.create_publisher(SensorHealth, STATE_HEALTH, 10)
        self._tf_pub = self.create_publisher(TFMessage, TF, 10)

        # Canonical ZIT6 topics are the new adapter contract.  The two legacy
        # subscriptions keep existing Foxy firmware usable until its topic
        # strings are rebuilt; both feeds enter the same estimator boundary.
        self.create_subscription(
            Float32MultiArray, ZIT6_POSITION, self._position_cb, 10)
        self.create_subscription(
            Float32MultiArray, LEGACY_ZIT6_POSITION, self._position_cb, 10)
        self.create_subscription(
            Float32MultiArray, ZIT6_VELOCITY, self._velocity_cb, 10)
        self.create_subscription(
            Float32MultiArray, LEGACY_ZIT6_VELOCITY, self._velocity_cb, 10)
        self.create_subscription(Empty, STATE_RESET, self._reset_cb, 10)
        self.create_subscription(DvlVelocity, DVL_VELOCITY, self._dvl_cb, 10)
        self.create_subscription(
            UsblMeasurement, USBL_MEASUREMENT, self._usbl_cb, 10)

        # IMU is used for angular velocity in both profiles.  Orientation is
        # intentionally not fused in this bootstrap adapter; FGO replaces it.
        self.create_subscription(Imu, IMU, self._imu_cb, 10)
        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.create_timer(1.0 / rate, self._publish_tick)
        self.get_logger().info(
            f'Localization boundary started ({"sim" if self._sim_mode else "real"} source)')

    @staticmethod
    def _parse_pose(data) -> _State | None:
        if len(data) < 4:
            return None
        values = list(data[:6]) if len(data) >= 6 else list(data[:4])
        if not _finite(values):
            return None
        pose = _State(x=float(data[0]), y=float(data[1]), z=float(data[2]))
        if len(data) >= 6:
            pose.roll = float(data[3])
            pose.pitch = float(data[4])
            pose.yaw = float(data[5])
        else:
            pose.yaw = float(data[3])
        return pose

    @staticmethod
    def _parse_velocity(data) -> tuple[float, float, float, float, float, float] | None:
        if len(data) < 4 or not _finite(data[:min(len(data), 6)]):
            return None
        values = [float(value) for value in data]
        values += [0.0] * (6 - len(values))
        return tuple(values[:6])

    def _position_cb(self, msg: Float32MultiArray) -> None:
        pose = self._parse_pose(msg.data)
        if pose is None:
            return
        with self._lock:
            self._raw_position = pose
            self._last_position_time = self.get_clock().now()
            self._have_measurement = True
            if self._map_origin is None:
                self._map_origin = _State(**pose.__dict__)

    def _velocity_cb(self, msg: Float32MultiArray) -> None:
        velocity = self._parse_velocity(msg.data)
        if velocity is None:
            return
        with self._lock:
            self._state.vx, self._state.vy, self._state.vz = velocity[:3]
            self._state.wx, self._state.wy, self._state.wz = velocity[3:]
            self._last_velocity_time = self.get_clock().now()

    def _dvl_cb(self, msg: DvlVelocity) -> None:
        if not msg.valid or not _finite((msg.velocity.x, msg.velocity.y, msg.velocity.z)):
            return
        with self._lock:
            vx = float(msg.velocity.x)
            vy = float(msg.velocity.y)
            vz = float(msg.velocity.z)
            if self._sim_mode:
                # v_sensor = v_reference + omega x r_sensor.  Remove the
                # angular contribution before feeding the dead reckoner.
                vx, vy, vz = _remove_sensor_lever_arm(
                    (vx, vy, vz),
                    (self._state.wx, self._state.wy, self._state.wz),
                    _SIM_DVL_POSITION_BODY,
                )
            self._state.vx = vx
            self._state.vy = vy
            self._state.vz = vz
            self._last_velocity_time = self.get_clock().now()
            self._have_measurement = True

    def _imu_cb(self, msg) -> None:
        with self._lock:
            values = (msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z)
            if _finite(values):
                self._state.wx, self._state.wy, self._state.wz = map(float, values)
                self._last_imu_time = self.get_clock().now()
                self._have_measurement = True

    def _usbl_cb(self, msg: UsblMeasurement) -> None:
        """Record USBL availability; the future FGO backend will fuse it."""
        if not msg.valid:
            return
        values = (msg.position.x, msg.position.y, msg.position.z)
        if _finite(values):
            with self._lock:
                self._have_measurement = True

    def _reset_cb(self, _msg: Empty) -> None:
        with self._lock:
            source = self._raw_position
            if source is None:
                self._state = _State()
                self._map_origin = None
                self._have_measurement = self._sim_mode
                return
            self._map_origin = _State(**source.__dict__)
            self._state.x = self._state.y = self._state.z = 0.0
            self._state.roll = self._state.pitch = self._state.yaw = 0.0
        self.get_logger().info('Estimator odom origin reset from STATE_RESET')

    def _update_real(self) -> None:
        if self._raw_position is None or self._map_origin is None:
            return
        raw, origin = self._raw_position, self._map_origin
        dx, dy = raw.x - origin.x, raw.y - origin.y
        cy, sy = math.cos(origin.yaw), math.sin(origin.yaw)
        self._state.x = cy * dx + sy * dy
        self._state.y = -sy * dx + cy * dy
        self._state.z = raw.z - origin.z
        self._state.roll = raw.roll - origin.roll
        self._state.pitch = raw.pitch - origin.pitch
        self._state.yaw = _wrap_rad(raw.yaw - origin.yaw)

    def _publish_tick(self) -> None:
        now = self.get_clock().now()
        with self._lock:
            dt = max(0.0, min(0.2, (now - self._last_update_time).nanoseconds / 1e9))
            self._last_update_time = now
            if not self._sim_mode:
                self._update_real()
            else:
                # A SIL estimator integrates DVL/IMU streams because there is
                # no raw position feed.  Do not integrate the last velocity
                # forever when Stonefish or a sensor publisher has stopped.
                # After the timeout, freeze the corresponding motion channel
                # and report localization unhealthy to downstream monitors.
                velocity_age = (
                    float('inf') if self._last_velocity_time is None else
                    (now - self._last_velocity_time).nanoseconds / 1e9)
                imu_age = (
                    float('inf') if self._last_imu_time is None else
                    (now - self._last_imu_time).nanoseconds / 1e9)
                velocity_fresh = velocity_age <= self._position_timeout
                imu_fresh = imu_age <= self._position_timeout
                if not velocity_fresh:
                    self._state.vx = self._state.vy = self._state.vz = 0.0
                if not imu_fresh:
                    self._state.wx = self._state.wy = self._state.wz = 0.0
                if not velocity_fresh or not imu_fresh:
                    if not self._sim_sensor_stale_reported:
                        self.get_logger().warning(
                            '仿真定位传感器超时；停止使用旧速度积分')
                        self._sim_sensor_stale_reported = True
                elif self._sim_sensor_stale_reported:
                    self.get_logger().info('仿真定位传感器恢复')
                    self._sim_sensor_stale_reported = False
                dx, dy = _body_to_world(self._state.vx, self._state.vy, self._state.yaw)
                self._state.x += dx * dt
                self._state.y += dy * dt
                self._state.z += self._state.vz * dt
                self._state.yaw = _wrap_rad(self._state.yaw + self._state.wz * dt)
            state = _State(**self._state.__dict__)
            healthy = self._have_measurement
            if self._sim_mode:
                velocity_age = (
                    float('inf') if self._last_velocity_time is None else
                    (now - self._last_velocity_time).nanoseconds / 1e9)
                imu_age = (
                    float('inf') if self._last_imu_time is None else
                    (now - self._last_imu_time).nanoseconds / 1e9)
                healthy = healthy and (
                    velocity_age <= self._position_timeout and
                    imu_age <= self._position_timeout)
            elif self._last_position_time is not None:
                age = (now - self._last_position_time).nanoseconds / 1e9
                healthy = healthy and age <= self._position_timeout

        stamp = now.to_msg()
        pose = PoseInfo()
        pose.stamp = stamp
        with self._lock:
            origin = self._map_origin
        if origin is not None:
            pose.origin_x = origin.x
            pose.origin_y = origin.y
            pose.origin_z = origin.z
            pose.origin_yaw = math.degrees(origin.yaw)
        pose.robot_x = state.x
        pose.robot_y = state.y
        pose.robot_z = state.z
        pose.robot_roll = math.degrees(state.roll)
        pose.robot_pitch = math.degrees(state.pitch)
        pose.robot_yaw = math.degrees(state.yaw)
        self._odom_pub.publish(pose)

        twist = TwistWithCovarianceStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = 'base_link'
        twist.twist.twist.linear.x = state.vx
        twist.twist.twist.linear.y = state.vy
        twist.twist.twist.linear.z = state.vz
        twist.twist.twist.angular.x = state.wx
        twist.twist.twist.angular.y = state.wy
        twist.twist.twist.angular.z = state.wz
        self._twist_pub.publish(twist)

        health = SensorHealth()
        health.header.stamp = stamp
        health.header.frame_id = 'base_link'
        health.sensor_name = 'localization'
        health.available = bool(healthy)
        health.quality = 1.0 if healthy else 0.0
        health.detail = (
            'bootstrap dead reckoning' if self._sim_mode and healthy
            else 'simulation sensor timeout' if self._sim_mode
            else 'ZIT6 navigation feedback')
        self._health_pub.publish(health)

        if self._publish_tf_enabled:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            transform.transform.translation.x = state.x
            transform.transform.translation.y = state.y
            transform.transform.translation.z = state.z
            half = state.yaw / 2.0
            transform.transform.rotation.z = math.sin(half)
            transform.transform.rotation.w = math.cos(half)
            self._tf_pub.publish(TFMessage(transforms=[transform]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EstimatorNode()
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
