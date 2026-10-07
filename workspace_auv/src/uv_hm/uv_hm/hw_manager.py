"""Hardware manager node: command adaptation, state monitoring.

Responsibilities:
- Forward the BasicMotion arm heartbeat to /zit6/cmd/agxhbt
- Forward canonical servo/light commands to the firmware endpoints
- Subscribe to /auv/hardware/zit6/state/status, heartbeat, and thruster state
- Adapt legacy servo target state into the canonical hardware namespace
- Parse and log MCU state in human-readable format
- Watchdog: heartbeat timeout (7s), battery low, error flags, thrust sat
- INS startup sequence tracking
"""

from __future__ import annotations

import threading
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from std_msgs.msg import Float32MultiArray, UInt8, UInt32

from zit6_interfaces.msg import ZitOdom, ZitServo, ZitServoState, ZitSetpoint, ZitStatus
from zit6_interfaces.srv import GetParams, SetOrigin, UpdateParams
from zit6_interfaces.msg import ZitUsbl
from uv_msgs.msg import UsblMeasurement
from auv_protocol.topics import (
    LEGACY_ZIT6_HEARTBEAT, LEGACY_ZIT6_ODOM, LEGACY_ZIT6_SET_ORIGIN,
    LEGACY_ZIT6_SETPOINT, LEGACY_ZIT6_SIM_NAV,
    LEGACY_ZIT6_GET_PARAMS, LEGACY_ZIT6_UPDATE_PARAMS,
    LEGACY_ZIT6_HEARTBEAT_STATE, LEGACY_ZIT6_POSITION,
    LEGACY_ZIT6_LIGHT, LEGACY_ZIT6_SERVO, LEGACY_ZIT6_SERVO_STATE,
    LEGACY_ZIT6_STATUS, LEGACY_ZIT6_THRUSTER, LEGACY_ZIT6_USBL,
    LEGACY_ZIT6_VELOCITY,
    ZIT6_STATUS, ZIT6_HEARTBEAT_STATE, ZIT6_THRUSTER,
    ZIT6_ARM_HEARTBEAT, ZIT6_ODOM, ZIT6_SET_ORIGIN, ZIT6_SETPOINT, ZIT6_SIM_NAV,
    ZIT6_GET_PARAMS, ZIT6_UPDATE_PARAMS,
    ZIT6_LIGHT, ZIT6_POSITION, ZIT6_SERVO, ZIT6_SERVO_STATE, ZIT6_VELOCITY,
    USBL_MEASUREMENT,
)


# ── INS state → human-readable name ─────────────────────────────
_INS_NAMES = {
    0: '待机',
    1: '粗对准',
    2: '精对准',
    3: 'SINS/GPS/DVL',
    4: 'SINS/DVL',
    5: 'MRU',
}

# ── Control level → human-readable name ─────────────────────────
_CTRL_NAMES = {
    0: 'NONE',
    1: 'POS',
    2: 'VEL',
    3: 'FORCE',
}

# ── Error flag bit definitions (matching ZitStatus constants) ───
_ERROR_BITS = [
    (1 << 0, 'FORCE_STOP'),
    (1 << 1, 'SENSOR_FAIL'),
    (1 << 2, 'VOLTAGE_LOW'),
    (1 << 3, 'COMM_TIMEOUT'),
]


class HwManagerNode(Node):
    """Hardware manager: transport adapter and state monitor."""

    # ── Startup phase enum ───────────────────────────────────────
    PHASE_INIT = 'INIT'
    PHASE_WAITING_INS = 'WAITING_INS'
    PHASE_INS_READY = 'INS_READY'
    PHASE_ARMED = 'ARMED'
    PHASE_RUNNING = 'RUNNING'
    PHASE_DEGRADED = 'DEGRADED'

    def __init__(self):
        super().__init__('hw_manager')

        # ── Parameters ───────────────────────────────────────────
        self.declare_parameter('watchdog_timeout', 7.0)
        self.declare_parameter('setorigin_timeout', 2.0)
        self.declare_parameter('parameter_service_timeout', 2.0)
        self.declare_parameter('battery_low_threshold', 14.0)
        self.declare_parameter('cycle_time_warn_threshold', 100.0)
        self.declare_parameter(
            'legacy_state_topics', True,
            ParameterDescriptor(
                description='Bridge the current /zit6/state/* firmware topics'))

        # ── Internal state ───────────────────────────────────────
        self._status_lock = threading.Lock()
        self._mcu_status = ZitStatus()
        self._last_mcu_hb_seq = 0
        self._last_mcu_hb_time = self.get_clock().now()
        self._last_status_time = self.get_clock().now()
        self._last_thr_time = self.get_clock().now()
        self._last_thr_forces = [0.0] * 6
        self._startup_phase = self.PHASE_INIT
        self._prev_armed = False

        # BasicMotion owns arming; this node only forwards each command.
        self._heartbeat_pub = self.create_publisher(
            UInt32, LEGACY_ZIT6_HEARTBEAT, 10)
        self._heartbeat_sub = self.create_subscription(
            UInt32, ZIT6_ARM_HEARTBEAT, self._heartbeat_cb, 10)
        self._setpoint_command_pub = self.create_publisher(
            ZitSetpoint, LEGACY_ZIT6_SETPOINT, 10)
        self._setpoint_command_sub = self.create_subscription(
            ZitSetpoint, ZIT6_SETPOINT, self._setpoint_command_cb, 10)
        self._sim_nav_pub = self.create_publisher(
            Float32MultiArray, LEGACY_ZIT6_SIM_NAV, 10)
        self._sim_nav_sub = self.create_subscription(
            Float32MultiArray, ZIT6_SIM_NAV, self._sim_nav_cb, 10)

        self._origin_proxy_lock = threading.Lock()
        self._pending_origin_proxy = None
        origin_group = ReentrantCallbackGroup()
        self._origin_client = self.create_client(
            SetOrigin, LEGACY_ZIT6_SET_ORIGIN, callback_group=origin_group)
        self._origin_service = self.create_service(
            SetOrigin, ZIT6_SET_ORIGIN, self._set_origin_cb,
            callback_group=origin_group)
        self._origin_timeout_timer = self.create_timer(0.05, self._origin_timeout_cb)
        self._param_proxy_lock = threading.Lock()
        self._pending_param_proxies = {}
        self._param_clients = {
            'get': self.create_client(
                GetParams, LEGACY_ZIT6_GET_PARAMS, callback_group=origin_group),
            'update': self.create_client(
                UpdateParams, LEGACY_ZIT6_UPDATE_PARAMS, callback_group=origin_group),
        }
        self._get_params_service = self.create_service(
            GetParams, ZIT6_GET_PARAMS, self._get_params_cb, callback_group=origin_group)
        self._update_params_service = self.create_service(
            UpdateParams, ZIT6_UPDATE_PARAMS, self._update_params_cb,
            callback_group=origin_group)
        self._params_timeout_timer = self.create_timer(0.05, self._params_timeout_cb)

        # ── MCU state subscriptions ──────────────────────────────
        self._state_publishers = {
            'status': self.create_publisher(ZitStatus, ZIT6_STATUS, 10),
            'odom': self.create_publisher(ZitOdom, ZIT6_ODOM, 10),
            'servo': self.create_publisher(
                ZitServoState, ZIT6_SERVO_STATE, 10),
            'position': self.create_publisher(
                Float32MultiArray, ZIT6_POSITION, 10),
            'velocity': self.create_publisher(
                Float32MultiArray, ZIT6_VELOCITY, 10),
            'thruster': self.create_publisher(
                Float32MultiArray, ZIT6_THRUSTER, 10),
            'heartbeat': self.create_publisher(
                UInt32, ZIT6_HEARTBEAT_STATE, 10),
            'usbl': self.create_publisher(
                UsblMeasurement, USBL_MEASUREMENT, 10),
        }

        # Canonical actuator commands are adapted to the firmware endpoints.
        self._servo_command_pub = self.create_publisher(
            ZitServo, LEGACY_ZIT6_SERVO, 10)
        self._light_command_pub = self.create_publisher(
            UInt8, LEGACY_ZIT6_LIGHT, 10)
        self._servo_command_sub = self.create_subscription(
            ZitServo, ZIT6_SERVO, self._servo_command_cb, 10)
        self._light_command_sub = self.create_subscription(
            UInt8, ZIT6_LIGHT, self._light_command_cb, 10)

        if bool(self.get_parameter('legacy_state_topics').value):
            self._odom_sub = self.create_subscription(
                ZitOdom, LEGACY_ZIT6_ODOM, self._legacy_odom_cb, 10)
            self._status_sub = self.create_subscription(
                ZitStatus, LEGACY_ZIT6_STATUS,
                self._legacy_status_cb, 10)
            self._mcu_hb_sub = self.create_subscription(
                UInt32, LEGACY_ZIT6_HEARTBEAT_STATE,
                self._legacy_mcu_hb_cb, 10)
            self._thr_sub = self.create_subscription(
                Float32MultiArray, LEGACY_ZIT6_THRUSTER,
                self._legacy_thr_cb, 10)
            self._position_sub = self.create_subscription(
                Float32MultiArray, LEGACY_ZIT6_POSITION,
                self._legacy_position_cb, 10)
            self._velocity_sub = self.create_subscription(
                Float32MultiArray, LEGACY_ZIT6_VELOCITY,
                self._legacy_velocity_cb, 10)
            self._usbl_sub = self.create_subscription(
                ZitUsbl, LEGACY_ZIT6_USBL,
                self._legacy_usbl_cb, 10)
            self._servo_state_sub = self.create_subscription(
                ZitServoState, LEGACY_ZIT6_SERVO_STATE,
                self._legacy_servo_state_cb, 10)
        else:
            self._odom_sub = None
            self._status_sub = self.create_subscription(
                ZitStatus, ZIT6_STATUS, self._status_cb, 10)
            self._mcu_hb_sub = self.create_subscription(
                UInt32, ZIT6_HEARTBEAT_STATE, self._mcu_hb_cb, 10)
            self._thr_sub = self.create_subscription(
                Float32MultiArray, ZIT6_THRUSTER, self._thr_cb, 10)
            self._position_sub = None
            self._velocity_sub = None
            self._usbl_sub = self.create_subscription(
                UsblMeasurement, USBL_MEASUREMENT, lambda msg: None, 10)

        # ── Timers ───────────────────────────────────────────────
        self._summary_timer = self.create_timer(1.0, self._summary_cb)

        self.get_logger().info('HW Manager started')
        self.get_logger().info(
            f'  heartbeat adapted {ZIT6_ARM_HEARTBEAT} -> {LEGACY_ZIT6_HEARTBEAT}')
        self.get_logger().info(
            f'  watchdog_timeout='
            f'{self.get_parameter("watchdog_timeout").value}s')
        self._startup_phase = self.PHASE_WAITING_INS

    # ── Heartbeat ────────────────────────────────────────────────

    def _heartbeat_cb(self, message: UInt32):
        """Forward exactly the owner-supplied heartbeat, without a local timer."""
        self._heartbeat_pub.publish(message)

    def _setpoint_command_cb(self, message: ZitSetpoint):
        self._setpoint_command_pub.publish(message)

    def _sim_nav_cb(self, message: Float32MultiArray):
        self._sim_nav_pub.publish(message)

    def _legacy_odom_cb(self, message: ZitOdom):
        self._state_publishers['odom'].publish(message)

    async def _set_origin_cb(self, _request, response):
        """Proxy one asynchronous MCU call; never spin inside a callback."""
        with self._origin_proxy_lock:
            if self._pending_origin_proxy is not None:
                response.success = False
                response.message = 'setorigin busy'
                return response
            if not self._origin_client.service_is_ready():
                response.success = False
                response.message = 'MCU setorigin service unavailable'
                return response
            try:
                future = self._origin_client.call_async(SetOrigin.Request())
            except Exception as error:
                response.success = False
                response.message = str(error)[:64]
                return response
            completion = Future(executor=self.executor)
            pending = {
                'future': future, 'completion': completion, 'response': response,
                'deadline': time.monotonic() + float(
                    self.get_parameter('setorigin_timeout').value),
            }
            self._pending_origin_proxy = pending
        future.add_done_callback(lambda result: self._origin_proxy_done(pending, result))
        return await completion

    def _origin_proxy_done(self, pending, future):
        with self._origin_proxy_lock:
            if self._pending_origin_proxy is not pending:
                return
            self._pending_origin_proxy = None
        response = pending['response']
        try:
            result = future.result()
            if result is None:
                raise RuntimeError('MCU setorigin returned no response')
            response.success = bool(result.success)
            response.message = str(result.message)[:64]
            response.origin_nav = [float(value) for value in result.origin_nav]
            response.nav_timestamp_ms = int(result.nav_timestamp_ms)
            response.origin_generation = int(result.origin_generation)
        except Exception as error:
            response.success = False
            response.message = str(error)[:64]
        pending['completion'].set_result(response)

    def _origin_timeout_cb(self):
        with self._origin_proxy_lock:
            pending = self._pending_origin_proxy
            if pending is None or time.monotonic() < pending['deadline']:
                return
            self._pending_origin_proxy = None
        response = pending['response']
        response.success = False
        response.message = 'MCU setorigin response timeout'
        pending['completion'].set_result(response)
        if not pending['future'].done():
            self._origin_client.remove_pending_request(pending['future'])
            pending['future'].cancel()

    async def _get_params_cb(self, request, response):
        return await self._proxy_params('get', request, response)

    async def _update_params_cb(self, request, response):
        return await self._proxy_params('update', request, response)

    async def _proxy_params(self, name, request, response):
        with self._param_proxy_lock:
            if name in self._pending_param_proxies:
                response.success = False
                response.message = 'MCU parameter service busy'
                return response
            client = self._param_clients[name]
            if not client.service_is_ready():
                response.success = False
                response.message = 'MCU parameter service unavailable'
                return response
            try:
                future = client.call_async(request)
            except Exception as error:
                response.success = False
                response.message = str(error)
                return response
            completion = Future(executor=self.executor)
            pending = {
                'client': client, 'future': future,
                'completion': completion, 'response': response,
                'deadline': time.monotonic() + float(
                    self.get_parameter('parameter_service_timeout').value),
            }
            self._pending_param_proxies[name] = pending
        future.add_done_callback(
            lambda result: self._params_proxy_done(name, pending, result))
        return await completion

    def _params_proxy_done(self, name, pending, future):
        with self._param_proxy_lock:
            if self._pending_param_proxies.get(name) is not pending:
                return
            del self._pending_param_proxies[name]
        response = pending['response']
        try:
            result = future.result()
            if result is None:
                raise RuntimeError('MCU parameter service returned no response')
            for field in response.get_fields_and_field_types():
                setattr(response, field, getattr(result, field))
        except Exception as error:
            response.success = False
            response.message = str(error)
        pending['completion'].set_result(response)

    def _params_timeout_cb(self):
        now = time.monotonic()
        expired = []
        with self._param_proxy_lock:
            for name, pending in list(self._pending_param_proxies.items()):
                if now >= pending['deadline']:
                    del self._pending_param_proxies[name]
                    expired.append(pending)
        for pending in expired:
            response = pending['response']
            response.success = False
            response.message = 'MCU parameter service response timeout'
            pending['completion'].set_result(response)
            if not pending['future'].done():
                pending['client'].remove_pending_request(pending['future'])
                pending['future'].cancel()

    def _servo_command_cb(self, msg: ZitServo):
        """Forward the canonical servo command to the firmware topic."""
        self._servo_command_pub.publish(msg)

    def _light_command_cb(self, msg: UInt8):
        """Forward the canonical light command to the firmware topic."""
        self._light_command_pub.publish(msg)

    # ── State callbacks ──────────────────────────────────────────

    def _status_cb(self, msg: ZitStatus):
        """Receive ZitStatus from MCU (10Hz). Run startup state machine."""
        with self._status_lock:
            old_phase = self._startup_phase
            self._mcu_status = msg
            self._last_status_time = self.get_clock().now()

            # Detect unexpected disarm
            if self._prev_armed and not msg.is_armed:
                self.get_logger().error(
                    'MCU unexpectedly DISARMED! '
                    f'(was armed, now locked, error_flags=0x{msg.error_flags:08X})')
            self._prev_armed = msg.is_armed

            # ── Startup state machine ──
            if self._startup_phase == self.PHASE_WAITING_INS:
                if msg.navigation_ready:
                    self._startup_phase = self.PHASE_INS_READY
                    ins_name = _INS_NAMES.get(msg.ins_state, f'?({msg.ins_state})')
                    self.get_logger().info(
                        f'INS navigation ready! state={ins_name}')
            elif self._startup_phase == self.PHASE_INS_READY:
                if msg.is_armed:
                    self._startup_phase = self.PHASE_ARMED
                    self.get_logger().info(
                        f'MCU ARMED! arm_mode={msg.arm_mode}')
            elif self._startup_phase == self.PHASE_ARMED:
                if msg.control_level > 0:
                    self._startup_phase = self.PHASE_RUNNING
                    ctrl_name = _CTRL_NAMES.get(msg.control_level,
                                                f'?({msg.control_level})')
                    self.get_logger().info(
                        f'MCU RUNNING: control_level={ctrl_name}')

    def _legacy_status_cb(self, msg: ZitStatus):
        """Adapt the current firmware status topic into the /auv contract."""
        self._state_publishers['status'].publish(msg)
        self._status_cb(msg)

    def _mcu_hb_cb(self, msg: UInt32):
        """Receive MCU heartbeat sequence number (1Hz)."""
        with self._status_lock:
            old_seq = self._last_mcu_hb_seq
            self._last_mcu_hb_seq = msg.data
            self._last_mcu_hb_time = self.get_clock().now()

        if old_seq != 0 and msg.data == old_seq:
            # Sequence stalled — MCU may be hung
            self.get_logger().warn(
                f'MCU heartbeat seq stalled at {msg.data}',
                throttle_duration_sec=10.0)

    def _legacy_mcu_hb_cb(self, msg: UInt32):
        self._state_publishers['heartbeat'].publish(msg)
        self._mcu_hb_cb(msg)

    def _thr_cb(self, msg: Float32MultiArray):
        """Monitor thruster forces (30Hz). Warn on saturation."""
        if len(msg.data) >= 6:
            self._last_thr_forces = list(msg.data[:6])
            self._last_thr_time = self.get_clock().now()
            max_thrust = max(abs(v) for v in msg.data[:6])
            if max_thrust > 0.95:
                t_str = ', '.join(f'{t:.2f}' for t in msg.data[:6])
                self.get_logger().warn(
                    f'Thruster saturation: max={max_thrust:.3f} '
                    f'thrusts=[{t_str}]',
                    throttle_duration_sec=5.0)

    def _legacy_thr_cb(self, msg: Float32MultiArray):
        self._state_publishers['thruster'].publish(msg)
        self._thr_cb(msg)

    def _legacy_position_cb(self, msg: Float32MultiArray):
        self._state_publishers['position'].publish(msg)

    def _legacy_velocity_cb(self, msg: Float32MultiArray):
        self._state_publishers['velocity'].publish(msg)

    def _legacy_servo_state_cb(self, msg: ZitServoState):
        """Adapt the firmware's accepted servo targets to the /auv contract."""
        self._state_publishers['servo'].publish(msg)

    def _legacy_usbl_cb(self, msg: ZitUsbl):
        """Adapt the embedded USBL frame into the canonical measurement msg."""
        measurement = UsblMeasurement()
        measurement.header.stamp = self.get_clock().now().to_msg()
        measurement.header.frame_id = 'usbl_link'
        measurement.position.x = float(msg.beacon_north_m)
        measurement.position.y = float(msg.beacon_east_m)
        measurement.position.z = float(msg.beacon_depth_m)
        measurement.position_covariance = [0.0] * 9
        measurement.valid = bool(msg.sensor_status & (1 << 5))
        self._state_publishers['usbl'].publish(measurement)

    # ── Periodic timers ──────────────────────────────────────────

    def _summary_cb(self):
        """1Hz: unified status summary + watchdog checks."""
        now = self.get_clock().now()
        timeout = self.get_parameter('watchdog_timeout').value

        with self._status_lock:
            s = self._mcu_status
            hb_seq = self._last_mcu_hb_seq
            last_hb = self._last_mcu_hb_time
            last_status = self._last_status_time
            thr_forces = list(self._last_thr_forces)

        # ── Watchdog checks (inline, event-driven) ────────────
        hb_elapsed = (now - last_hb).nanoseconds / 1e9
        status_elapsed = (now - last_status).nanoseconds / 1e9

        if hb_elapsed > timeout:
            self.get_logger().error(
                f'MCU heartbeat lost: {hb_elapsed:.0f}s (timeout={timeout}s)',
                throttle_duration_sec=5.0)
            self._startup_phase = self.PHASE_DEGRADED

        if status_elapsed > timeout:
            self.get_logger().error(
                f'MCU status lost: {status_elapsed:.0f}s',
                throttle_duration_sec=5.0)

        # ── Battery warning ───────────────────────────────────
        low_thresh = self.get_parameter('battery_low_threshold').value
        if 0.1 < s.battery_voltage < low_thresh:
            self.get_logger().warn(
                f'Battery low: {s.battery_voltage:.1f}V < {low_thresh}V',
                throttle_duration_sec=30.0)

        # ── Error flags ────────────────────────────────────────
        if s.error_flags != 0:
            err_names = [name for mask, name in _ERROR_BITS if s.error_flags & mask]
            self.get_logger().error(
                f'MCU errors: 0x{s.error_flags:08X} → {", ".join(err_names)}',
                throttle_duration_sec=5.0)

        # ── Build one-line summary ─────────────────────────────
        armed = '●' if s.is_armed else '○'
        nav_ok = '✓' if s.navigation_ready else '✗'
        ins = _INS_NAMES.get(s.ins_state, f'?{s.ins_state}')
        ctrl = _CTRL_NAMES.get(s.control_level, f'?{s.control_level}')
        phase = self._startup_phase

        # Forces: only show non-zero
        force_parts = []
        labels = ['Fx', 'Fy', 'Fz', 'Mz']
        for i, f in enumerate(thr_forces[:4]):
            if abs(f) > 0.01:
                force_parts.append(f'{labels[i]}={f:+.1f}')
        forces = ' '.join(force_parts) if force_parts else 'idle'

        self.get_logger().info(
            f'ARM:{armed} {phase:12s} | '
            f'INS:{ins:14s} NAV:{nav_ok} | '
            f'CTRL:{ctrl:4s} | '
            f'BAT:{s.battery_voltage:4.1f}V | '
            f'CYC:{s.cycle_time_ms:5.1f}ms | '
            f'SEQ:{hb_seq:4d} | '
            f'{forces}')


def main(args=None):
    rclpy.init(args=args)
    node = HwManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('HW Manager shutting down (KeyboardInterrupt)')
    except Exception as e:
        node.get_logger().fatal(f'HW Manager crashed: {e}')
        import traceback
        traceback.print_exc()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
