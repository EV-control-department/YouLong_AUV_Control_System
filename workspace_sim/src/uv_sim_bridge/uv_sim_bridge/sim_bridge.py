"""Simulation bridge: emulate the ZIT6 MCU firmware, driven by the native C++ control core.

Protocol matches real AUV: ZitSetpoint in → ZitOdom + ZitStatus out.
Unlike the old homegrown Python cascade PID, the 100Hz control loop runs in the
native ZIT6 control core (zit6_control_core.Zit6Controller) — a host compile of
the actual firmware cascade controller.
Thruster mixing (YouLong six-thruster geometry) happens here in Python, since the
real firmware leaves that to the external motor controller board.
"""

from __future__ import annotations

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Imu
from std_msgs.msg import UInt8, UInt32, Float32MultiArray, String

from zit6_interfaces.msg import ZitOdom, ZitServo, ZitSetpoint, ZitStatus
from zit6_interfaces.srv import GetParams, SetOrigin, UpdateParams
from uv_msgs.msg import DvlAltitude, DvlVelocity
from auv_protocol.topics import (
    DVL_ALTITUDE, DVL_VELOCITY, IMU, PRESSURE,
    ZIT6_GET_PARAMS, ZIT6_UPDATE_PARAMS,
    ZIT6_ARM_HEARTBEAT, ZIT6_ODOM, ZIT6_SET_ORIGIN,
    ZIT6_INS, ZIT6_LIGHT, ZIT6_SERVO, ZIT6_SETPOINT,
    ZIT6_STATUS, ZIT6_THRUSTER,
    ZIT6_HEARTBEAT_STATE, ZIT6_SIM_NAV,
    SIM_CONTROL_PERFORMANCE,
)

from uv_sim_bridge.camera_adapter import CameraAdapter
from uv_sim_bridge.sensor_adapter import SensorAdapter
from uv_sim_bridge.thrust_mixer import ThrustMixer
from uv_sim_bridge.zit6_emulator import Zit6Emulator
from uv_sim_bridge.actuator_adapter import ActuatorAdapter
from uv_sim_bridge.performance import ControlLoopStats
from uv_sim_bridge.raw_navigation import RawNavigation
from uv_sim_bridge.arm_lifecycle import ArmLifecycle


def _wrap_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _as_bool(value) -> bool:
    """Parse ROS launch booleans safely, including string substitutions."""
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _find_firmware_config(overrides=None) -> dict:
    """定位配置,返回其 chassis 段(或 overrides)。

    仿真侧优先读独立配置 workspace_sim/src/zit6_control_core/sim_config.json
    (与固件 config.json 格式一致);该文件不存在时回退读固件 config.json。
    """
    if overrides and "chassis" in overrides:
        return overrides["chassis"]

    # The bridge runs both directly on the host and inside the Docker
    # container.  Do not rely on the host-only absolute path: when the
    # container cannot see it, the old code silently constructed a controller
    # with an empty config (all PID gains became zero).
    source_root = Path(__file__).resolve().parents[2]
    candidates = [
        source_root / "zit6_control_core" / "sim_config.json",
        Path("/workspace/workspace_sim/src/zit6_control_core/sim_config.json"),
        Path("/workspace/src/zit6_control_core/sim_config.json"),
        Path.cwd() / "workspace_sim/src/zit6_control_core/sim_config.json",
        Path("/home/doc049/dev/UUV/YouLong_AUV_Control_System/workspace_sim/src/zit6_control_core/sim_config.json"),
        Path("/workspace/third_party/AUV_zit6_cmake/UserApp/Config/config.json"),
        Path("/home/doc049/dev/UUV/YouLong_AUV_Control_System/third_party/AUV_zit6_cmake/UserApp/Config/config.json"),
    ]
    for p in candidates:
        if p.exists():
            try:
                cfg = json.loads(p.read_text())
                if "chassis" in cfg:
                    return cfg["chassis"]
            except Exception:
                pass
    # 缺省用与固件一致的保守值(planner off)。
    return {}


class SimBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("sim_bridge")

        self.declare_parameter('hil_mode', False)
        self._hil_mode = _as_bool(self.get_parameter('hil_mode').value)
        self.declare_parameter('dvl_topic', DVL_VELOCITY)
        self.declare_parameter('imu_topic', IMU)
        self.declare_parameter('navigation_timeout', 2.0)
        self._boot_monotonic_s = time.monotonic()
        self._navigation = RawNavigation(
            float(self.get_parameter('navigation_timeout').value),
            timestamp_origin_s=self._boot_monotonic_s)
        self.declare_parameter('camera_stitch_fps', 10.0)
        self.declare_parameter('publish_raw_camera_topics', False)
        self._control_stop = threading.Event()
        self._shutdown_requested = False
        self.context.on_shutdown(self._on_context_shutdown)

        self.cam = CameraAdapter(
            self.get_parameter('camera_stitch_fps').value,
            publish_raw_views=_as_bool(
                self.get_parameter('publish_raw_camera_topics').value))
        self.cam.bind(self)

        if self._hil_mode:
            self._init_hil()
            self.get_logger().info("sim_bridge started in HIL mode (thrust mixing + nav feed)")
        else:
            self._init_full()
            self.get_logger().info("sim_bridge started (native ZIT6 core, YouLong mixer)")

    def _on_context_shutdown(self) -> None:
        """Quiesce the real-time thread before ROS handles are destroyed."""
        self._shutdown_requested = True
        self._control_stop.set()

    def _publish_while_running(self, publisher, message) -> None:
        """Publish while the context is valid, ignoring only shutdown races."""
        if self._shutdown_requested:
            return
        try:
            publisher.publish(message)
        except Exception:
            if not self._shutdown_requested and rclpy.ok():
                raise

    # ── Full SIL mode: native ZIT6 control core + mixing ────────────

    def _init_full(self) -> None:
        self._tick = 0
        self._last_status_publish_s = float('-inf')
        self._last_thruster_publish_s = float('-inf')
        self._last_odom_publish_s = float('-inf')
        self._control_stats = ControlLoopStats(target_hz=100.0)

        self.thrust = [0.0] * 6
        self.force_6dof = [0.0] * 6
        self._core_lock = threading.RLock()
        self._forces_lock = threading.Lock()

        # ARM / INS state
        self._lifecycle = ArmLifecycle()
        self._ins_state = 1
        self._ins_nav_ready = False
        self._ins_boot_time = self.get_clock().now()
        self._ins_align_request = None
        self._dvl_enabled = False

        # === Native control core ===
        chassis = _find_firmware_config()
        self._core = Zit6Emulator(chassis)
        self.get_logger().info(
            f"ZIT6 native core loaded, control_level="
            f"{self._core.control_level}")

        # === Publishers ===
        self.zit6_odom_pub = self.create_publisher(ZitOdom, ZIT6_ODOM, 10)
        self.zit6_status_pub = self.create_publisher(ZitStatus, ZIT6_STATUS, 10)
        self.zit6_thr_pub = self.create_publisher(Float32MultiArray, ZIT6_THRUSTER, 10)
        self.zit6_hbt_pub = self.create_publisher(UInt32, ZIT6_HEARTBEAT_STATE, 10)
        self.performance_pub = self.create_publisher(
            String, SIM_CONTROL_PERFORMANCE, 10)
        self.dvl_velocity_pub = self.create_publisher(
            DvlVelocity, DVL_VELOCITY, 10)
        self.dvl_altitude_pub = self.create_publisher(
            DvlAltitude, DVL_ALTITUDE, 10)

        # === Subscriptions ===
        self.create_subscription(ZitSetpoint, ZIT6_SETPOINT, self._setpoint_cb, 10)
        self.create_subscription(ZitServo, ZIT6_SERVO, self._servo_cb, 10)
        self.create_subscription(UInt8, ZIT6_LIGHT, self._light_cb, 10)
        self._init_navigation_inputs()
        self.sensor_adapter = SensorAdapter(
            self, self.dvl_velocity_pub, self.dvl_altitude_pub)
        self.create_subscription(FluidPressure, PRESSURE, self._pressure_cb, 10)
        self.create_subscription(
            UInt32, ZIT6_ARM_HEARTBEAT, self._agxhbt_cb, 10)
        self.create_subscription(UInt8, ZIT6_INS, self._ins_cb, 10)

        # === Services ===
        self.create_service(SetOrigin, ZIT6_SET_ORIGIN, self._set_origin_cb)
        self.create_service(GetParams, ZIT6_GET_PARAMS, self._get_params_cb)
        self.create_service(UpdateParams, ZIT6_UPDATE_PARAMS, self._update_params_cb)

        self._mixer = ThrustMixer(heave_factor=0.8)
        self._actuator = ActuatorAdapter(self, self._mixer)

        # zithbt ~1Hz (firmware publishes ms-tick ~1Hz; hw_manager watchdog 7s)
        self.create_timer(1.0, self._publish_zithbt)
        self.create_timer(1.0, self._publish_performance)

        # Start the 100Hz control thread (separated from camera/state executor so
        # image stitching never starves control).
        self._control_stop = threading.Event()
        self._control_thread = threading.Thread(
            target=self._control_loop, name="zit6_control", daemon=True)
        self._control_thread.start()

    # ── HIL mode: real MCU runs PID; bridge only mixes thrust + feeds nav ──

    def _init_hil(self) -> None:
        self.thrust = [0.0] * 6
        self.force_4dof = [0.0, 0.0, 0.0, 0.0]
        self._forces_lock = threading.Lock()
        self.dvl_velocity_pub = self.create_publisher(
            DvlVelocity, DVL_VELOCITY, 10)
        self.dvl_altitude_pub = self.create_publisher(
            DvlAltitude, DVL_ALTITUDE, 10)
        self.sensor_adapter = SensorAdapter(
            self, self.dvl_velocity_pub, self.dvl_altitude_pub)
        self.create_subscription(Float32MultiArray, ZIT6_THRUSTER, self._thrust_cb, 10)
        self._mixer = ThrustMixer()
        self._actuator = ActuatorAdapter(self, self._mixer)

        # External HIL navigation is continuous raw nav, never MCU odom.
        self.sim_nav_pub = self.create_publisher(Float32MultiArray, ZIT6_SIM_NAV, 10)
        self._hil_navigation_configured = False
        self._hil_status = None
        self._hil_status_time_s = None
        self._hil_config_future = None
        self._hil_config_next_attempt_s = 0.0
        self._hil_last_heartbeat_tick = None
        self._hil_config_client = self.create_client(UpdateParams, ZIT6_UPDATE_PARAMS)
        self.create_subscription(ZitStatus, ZIT6_STATUS, self._hil_status_cb, 10)
        self.create_subscription(
            UInt32, ZIT6_HEARTBEAT_STATE, self._hil_heartbeat_cb, 10)
        self._init_navigation_inputs()
        self.create_timer(1.0 / 30.0, self._publish_sim_nav)

    def _thrust_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            self._publish_thrust_from_6dof(*msg.data[:6])

    def _init_navigation_inputs(self) -> None:
        self.create_subscription(
            DvlVelocity, str(self.get_parameter('dvl_topic').value),
            self._dvl_cb, 10)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value), self._imu_cb, 10)

    def _dvl_cb(self, msg: DvlVelocity) -> None:
        if msg.valid:
            self._navigation.update_velocity(
                (msg.velocity.x, msg.velocity.y, msg.velocity.z), time.monotonic())

    def _imu_cb(self, msg: Imu) -> None:
        angular = msg.angular_velocity
        self._navigation.update_imu(
            (angular.x, angular.y, angular.z), time.monotonic())

    def _hil_status_cb(self, msg: ZitStatus) -> None:
        self._hil_status = msg
        self._hil_status_time_s = time.monotonic()

    def _hil_heartbeat_cb(self, msg: UInt32) -> None:
        tick = int(msg.data)
        if (self._hil_last_heartbeat_tick is not None
                and ((tick - self._hil_last_heartbeat_tick) & 0xFFFFFFFF) > 0x80000000):
            # Runtime external-nav configuration must be reapplied after MCU reboot.
            self._hil_navigation_configured = False
            if self._hil_config_future is not None:
                self._hil_config_future.cancel()
                self._hil_config_future = None
        self._hil_last_heartbeat_tick = tick

    def _configure_hil_navigation(self) -> None:
        if self._hil_navigation_configured:
            return
        if self._hil_config_future is not None:
            if not self._hil_config_future.done():
                return
            try:
                response = self._hil_config_future.result()
                self._hil_navigation_configured = bool(response and response.success)
                if not self._hil_navigation_configured:
                    self.get_logger().error('HIL external navigation config rejected')
            except Exception as error:
                self.get_logger().error(f'HIL navigation config failed: {error}')
            self._hil_config_future = None
            self._hil_config_next_attempt_s = time.monotonic() + 1.0
            return
        if time.monotonic() < self._hil_config_next_attempt_s:
            return
        if (self._hil_status is None or self._hil_status.is_armed
                or self._hil_status_time_s is None
                or time.monotonic() - self._hil_status_time_s > 2.0
                or not self._hil_config_client.service_is_ready()):
            return
        request = UpdateParams.Request()
        request.paths = ['simulation.hitl_enabled', 'simulation.sitl_enabled']
        request.values = ['false', 'true']
        self._hil_config_future = self._hil_config_client.call_async(request)

    def _publish_sim_nav(self) -> None:
        self._configure_hil_navigation()
        sample = self._navigation.snapshot(time.monotonic())
        if not self._hil_navigation_configured or not sample.valid:
            return
        nav = Float32MultiArray()
        nav.data = list(sample.position + sample.velocity)
        self._publish_while_running(self.sim_nav_pub, nav)

    # ── ZIT6 setpoint callback → native core ────────────────────────

    def _setpoint_cb(self, msg: ZitSetpoint) -> None:
        mode = int(msg.control_key & 0x03)   # 0=POS, 1=VEL, 2=ACTUATOR
        if mode >= 3:
            return
        is_body = bool(msg.control_key & 0x10)
        is_inc = bool(msg.control_key & 0x20)
        mask = int(msg.type_mask)
        # val6: [x, y, z, roll, pitch, yaw] — yaw is RADIANS on the wire.
        val6 = [msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw]

        if not all(math.isfinite(float(value)) for value in val6):
            return
        try:
            with self._core_lock:
                odom = self._core.get_odom_snapshot()
                if not self._lifecycle.armed or (mode in (0, 1) and not odom['nav_valid']):
                    return
                self._core.update_setpoint(mode, val6, mask, is_body, is_inc)
            self.get_logger().debug(
                f"Setpoint: level={mode} is_body={is_body} is_inc={is_inc} "
                f"mask={mask} val={[f'{v:.2f}' for v in val6]}")
        except Exception as e:
            self.get_logger().error(f"core.update_setpoint failed: {e}")

    def _servo_cb(self, msg: ZitServo) -> None:
        self.get_logger().info(
            f"Servo {msg.servo_id}: {msg.angle:.3f} rad")

    def _light_cb(self, msg: UInt8) -> None:
        colors = {1: "red", 2: "yellow", 3: "green"}
        name = colors.get(msg.data, f"unknown ({msg.data})")
        self.get_logger().info(f"Light: {name}")

    # ── Heartbeat / ARM ─────────────────────────────────────────────

    def _agxhbt_cb(self, msg: UInt32) -> None:
        with self._core_lock:
            odom = self._core.get_odom_snapshot()
            previously_armed = self._lifecycle.armed
            self._lifecycle.heartbeat(
                int(msg.data), time.monotonic(),
                origin_ready=odom['origin_initialized'],
                nav_ready=odom['nav_valid'])
            if self._lifecycle.armed and not previously_armed:
                self._core.reset_setpoints()

    def _set_origin_cb(self, _request, response):
        with self._core_lock:
            if self._lifecycle.armed:
                response.success = False
                response.message = 'vehicle must be disarmed'
                return response
            commit = self._core.try_set_origin(self._mcu_tick_ms(), 200)
            response.success = commit['success']
            response.message = ('origin set' if response.success
                                else 'navigation invalid or stale')
            if response.success:
                # Pre-reset heartbeats must not ARM before the new odom is confirmed.
                self._lifecycle.reset_arming_qualification()
                self._core.set_control_level(0)
                response.origin_nav = commit['origin_nav']
                response.nav_timestamp_ms = commit['nav_timestamp_ms']
                response.origin_generation = commit['origin_generation']
        return response

    def _mcu_tick_ms(self) -> int:
        return int((time.monotonic() - self._boot_monotonic_s) * 1000) & 0xFFFFFFFF

    def _publish_zithbt(self) -> None:
        """~1Hz heartbeat, data = coarse ms tick (matches firmware; avoids hw_manager 7s watchdog)."""
        msg = UInt32()
        msg.data = self._mcu_tick_ms()
        self._publish_while_running(self.zit6_hbt_pub, msg)

    # ── INS command ─────────────────────────────────────────────────

    def _ins_cb(self, msg: UInt8) -> None:
        cmd = msg.data
        if cmd == 1:
            self._dvl_enabled = True
            if self._ins_state == 5:
                self._ins_align_request = self.get_clock().now()
        elif cmd == 2:
            self._dvl_enabled = False
            self._ins_align_request = None
            if self._ins_state == 4:
                self._ins_state = 5
        elif cmd == 3:
            self._ins_state = 1
            self._ins_nav_ready = False
            self._dvl_enabled = False
            self._ins_align_request = None
            self._ins_boot_time = self.get_clock().now()
            # INS restart disarms while retaining the explicitly set origin.
            with self._core_lock:
                self._lifecycle.disarm()
                self._core.set_control_level(0)

    # ── Parameter services (get/update write through native core gains) ──

    def _get_params_cb(self, request, response):
        response.success = True
        response.config_json = "{}"
        return response

    def _update_params_cb(self, request, response):
        # Runtime PID gain write-through into the native core is possible via
        # configure_pid; kept minimal — map firmware 'chassis.pid.*' paths here.
        response.success = True
        response.message = "No-op (params live in firmware config.json)"
        return response

    # ── 100Hz control thread ────────────────────────────────────────

    def _control_loop(self) -> None:
        """Drive the native core at 100Hz: update_nav -> step -> mix -> publish states."""
        deadline = time.monotonic() + 0.01
        while rclpy.ok() and not self._shutdown_requested and not self._control_stop.is_set():
            scheduled = deadline
            if self._control_stop.wait(max(0.0, scheduled - time.monotonic())):
                break
            tick_started = time.monotonic()
            self._control_stats.record_tick(
                tick_started, late_by_s=tick_started - scheduled)
            self._control_tick()
            deadline = scheduled + 0.01
            # Do not burst stale control steps after an overloaded interval.
            if deadline < time.monotonic():
                deadline = time.monotonic() + 0.01

    def _publish_performance(self) -> None:
        """Expose measured control timing for SIL/HIL acceptance checks."""
        if not hasattr(self, '_control_stats'):
            return
        payload = self._control_stats.snapshot(reset=True)
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._publish_while_running(self.performance_pub, message)

    def _control_tick(self) -> None:
        if self._shutdown_requested or not rclpy.ok():
            return
        self._tick = (self._tick + 1) % 60

        now_s = time.monotonic()
        sample = self._navigation.snapshot(now_s)
        self._update_ins_alignment()
        try:
            with self._core_lock:
                self._core.update_nav(sample.position, sample.velocity,
                                      sample.timestamp_ms,
                                      sample.valid and self._ins_nav_ready)
                odom = self._core.get_odom_snapshot()
                previously_armed = self._lifecycle.armed
                if self._lifecycle.check(
                        now_s, origin_ready=odom['origin_initialized'],
                        nav_ready=odom['nav_valid']):
                    self._core.set_control_level(0)
                    self.get_logger().info('ARM heartbeat timed out; disarmed')
                if self._lifecycle.armed and not previously_armed:
                    self._core.reset_setpoints()
                allow_control = (self._lifecycle.armed
                                 and (odom['nav_valid'] or self._core.control_level == 3))
                forces = self._core.step() if allow_control else [0.0] * 6
        except Exception as error:
            self.get_logger().error(f'core.step failed: {error}')
            forces = [0.0] * 6

        with self._forces_lock:
            self.force_6dof = list(forces)
            self.thrust = self._mixer.mix6(*forces)

        # Keep control/actuation at 100Hz, but publish telemetry at rates that
        # are sufficient for navigation and task control.  Time-based gates
        # avoid the jitter of modulo counters when the control thread slips.
        publish_now = time.monotonic()
        if publish_now - self._last_odom_publish_s >= (1.0 / 30.0):
            self._publish_odom()
            self._last_odom_publish_s = publish_now
        if publish_now - self._last_status_publish_s >= 0.1:
            self._publish_state()
            self._last_status_publish_s = publish_now
        if publish_now - self._last_thruster_publish_s >= (1.0 / 30.0):
            self._publish_thr()
            self._last_thruster_publish_s = publish_now

        # Thrust to Stonefish (owned by the simulator actuator adapter).
        self._actuator.publish_thrust(self.thrust)

    def _publish_thrust_from_6dof(self, fx, fy, fz, mroll, mpitch, myaw) -> None:
        """Mix HIL forces from the canonical ZIT6 thruster-state topic."""
        with self._forces_lock:
            self.force_6dof = [fx, fy, fz, mroll, mpitch, myaw]
            self.thrust = self._mixer.mix6(fx, fy, fz, mroll, mpitch, myaw)
        self._actuator.publish_thrust(self.thrust)

    def _pressure_cb(self, msg: FluidPressure) -> None:
        pass

    # ── State publishing ────────────────────────────────────────────

    def _update_ins_alignment(self) -> None:
        now = self.get_clock().now()
        if self._ins_state == 1:
            elapsed = (now - self._ins_boot_time).nanoseconds / 1e9
            if elapsed >= 1.5:
                self._ins_state = 5
                self._ins_nav_ready = True
                self.get_logger().info("INS: coarse alignment done → MRU mode")
                if self._dvl_enabled:
                    self._ins_align_request = now
        if self._ins_state == 5 and self._ins_align_request is not None:
            elapsed = (now - self._ins_align_request).nanoseconds / 1e9
            if elapsed >= 1.5:
                self._ins_state = 4
                self._ins_align_request = None
                self.get_logger().info("INS: SINS/DVL alignment complete")

    def _publish_state(self) -> None:
        status = ZitStatus()
        with self._core_lock:
            odom = self._core.get_odom_snapshot()
            status.is_armed = self._lifecycle.armed
            status.arm_mode = self._lifecycle.arm_mode
            status.control_level = self._core.control_level
        status.ins_state = self._ins_state
        status.navigation_ready = odom['nav_valid']
        with self._forces_lock:
            f = list(self.force_6dof)
        status.forces = [f[0], f[1], f[2], f[3], f[4], f[5]]
        status.cycle_time_ms = 10.0
        status.battery_voltage = 16.8
        status.error_flags = 0
        self._publish_while_running(self.zit6_status_pub, status)
    def _publish_odom(self) -> None:
        with self._core_lock:
            snapshot = self._core.get_odom_snapshot()
        message = ZitOdom()
        for key, value in snapshot.items():
            setattr(message, key, value)
        self._publish_while_running(self.zit6_odom_pub, message)

    def _publish_thr(self) -> None:
        thr_msg = Float32MultiArray()
        with self._forces_lock:
            thr_msg.data = list(self.force_6dof)
        self._publish_while_running(self.zit6_thr_pub, thr_msg)

    # ── Utilities ───────────────────────────────────────────────────

    def destroy_node(self) -> None:
        # Stop the 100Hz control thread first and join it, so no native-call or
        # publisher runs after rclpy tears down (avoids 'terminate called
        # without an active exception' at shutdown).
        self._control_stop.set()
        if getattr(self, '_control_thread', None) is not None:
            self._control_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        # launch may already have shut down the default context while
        # delivering SIGINT.  try_shutdown keeps a normal Ctrl-C from
        # becoming an exit-code-1 failure.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
