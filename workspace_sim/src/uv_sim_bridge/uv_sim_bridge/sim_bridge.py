"""Simulation bridge: emulate the ZIT6 MCU firmware, driven by the native C++ control core.

Protocol matches real AUV: ZitSetpoint in → ZitStatus + Float32MultiArray out.
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
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Float32, UInt8, UInt32, Float32MultiArray, String

from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from zit6_interfaces.srv import GetParams, UpdateParams
from uv_msgs.msg import DvlAltitude, DvlVelocity, PoseInfo
from auv_protocol.topics import (
    DVL_ALTITUDE, DVL_VELOCITY, PRESSURE, STATE_ODOM, STATE_TWIST,
    ZIT6_GET_PARAMS, ZIT6_UPDATE_PARAMS,
    ZIT6_HEARTBEAT, ZIT6_INS, ZIT6_LIGHT, ZIT6_SERVO, ZIT6_SETPOINT,
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
        self._control_stats = ControlLoopStats(target_hz=100.0)

        # Internal state (host policy, mirrors firmware MicroRosPublisher semantics)
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}  # estimated NED, angles deg
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}  # estimated body, rad/s
        self.vel_world = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        self.thrust = [0.0] * 6
        self.force_6dof = [0.0] * 6
        self.current_pose_ready = False

        # ARM / INS state
        self._armed = True
        self._arm_mode = 3
        self._last_heartbeat_time = self.get_clock().now()
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
        self.create_subscription(Float32, ZIT6_SERVO, self._servo_cb, 10)
        self.create_subscription(UInt8, ZIT6_LIGHT, self._light_cb, 10)
        self.create_subscription(PoseInfo, STATE_ODOM, self._state_odom_cb, 10)
        self.create_subscription(
            TwistWithCovarianceStamped, STATE_TWIST,
            self._state_twist_cb, 10)
        self.sensor_adapter = SensorAdapter(
            self, self.dvl_velocity_pub, self.dvl_altitude_pub)
        self.create_subscription(FluidPressure, PRESSURE, self._pressure_cb, 10)
        self.create_subscription(UInt32, ZIT6_HEARTBEAT, self._agxhbt_cb, 10)
        self.create_subscription(UInt8, ZIT6_INS, self._ins_cb, 10)

        # === Services ===
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
        self._forces_lock = threading.Lock()
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

        # Nav aggregation for MCU
        self._sim_pos = [0.0] * 6
        self._sim_vel = [0.0] * 6
        self.sim_nav_pub = self.create_publisher(Float32MultiArray, ZIT6_SIM_NAV, 10)
        # HIL navigation is sourced from the canonical estimated state.  The
        # simulator ground-truth topic is intentionally not part of this
        # control/MCU feed.
        self.create_subscription(PoseInfo, STATE_ODOM, self._sim_nav_pose_cb, 10)
        self.create_subscription(
            TwistWithCovarianceStamped, STATE_TWIST,
            self._sim_nav_twist_cb, 10)

    def _thrust_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            self._publish_thrust_from_6dof(*msg.data[:6])

    def _sim_nav_pose_cb(self, msg: PoseInfo) -> None:
        """Forward the canonical estimated pose to the HIL MCU adapter."""
        values = (
            msg.robot_x, msg.robot_y, msg.robot_z,
            msg.robot_roll, msg.robot_pitch, msg.robot_yaw,
        )
        if not all(math.isfinite(float(value)) for value in values):
            return
        self._sim_pos[0:3] = [float(msg.robot_x), float(msg.robot_y),
                              float(msg.robot_z)]
        self._sim_pos[3:6] = [
            math.radians(float(msg.robot_roll)),
            math.radians(float(msg.robot_pitch)),
            math.radians(float(msg.robot_yaw)),
        ]
        self._publish_sim_nav()

    def _sim_nav_twist_cb(self, msg: TwistWithCovarianceStamped) -> None:
        """Forward estimator velocity to the HIL MCU adapter."""
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            return
        self._sim_vel[:] = [float(value) for value in values]
        self._publish_sim_nav()

    def _publish_sim_nav(self) -> None:
        nav = Float32MultiArray()
        nav.data = self._sim_pos + self._sim_vel
        self._publish_while_running(self.sim_nav_pub, nav)

    # ── ZIT6 setpoint callback → native core ────────────────────────

    def _setpoint_cb(self, msg: ZitSetpoint) -> None:
        mode = int(msg.control_key & 0x03)   # 0=POS, 1=VEL, 2=ACTUATOR
        is_body = bool(msg.control_key & 0x10)
        is_inc = bool(msg.control_key & 0x20)
        mask = int(msg.type_mask)
        # val6: [x, y, z, roll, pitch, yaw] — yaw is RADIANS on the wire.
        val6 = [msg.x, msg.y, msg.z, msg.roll, msg.pitch, msg.yaw]

        # Firmware gating: position/velocity setpoints need armed + nav_valid.
        # We stay always-armed in sim, so only pass through if pose ready.
        if mode in (0, 1) and not self.current_pose_ready:
            self.get_logger().debug("setpoint dropped: pose not ready")
            return
        try:
            self._core.update_setpoint(mode, val6, mask, is_body, is_inc)
            self.get_logger().debug(
                f"Setpoint: level={mode} is_body={is_body} is_inc={is_inc} "
                f"mask={mask} val={[f'{v:.2f}' for v in val6]}")
        except Exception as e:
            self.get_logger().error(f"core.update_setpoint failed: {e}")

    def _servo_cb(self, msg: Float32) -> None:
        self.get_logger().info(f"Servo: {msg.data:.3f} rad")

    def _light_cb(self, msg: UInt8) -> None:
        colors = {1: "red", 2: "yellow", 3: "green"}
        name = colors.get(msg.data, f"unknown ({msg.data})")
        self.get_logger().info(f"Light: {name}")

    # ── Heartbeat / ARM ─────────────────────────────────────────────

    def _agxhbt_cb(self, msg: UInt32) -> None:
        self._last_heartbeat_time = self.get_clock().now()
        arm_val = msg.data
        newly_armed = False
        if arm_val == 3:
            if not self._armed:
                self.get_logger().info("ARM: force arm (mode 3)")
                newly_armed = True
            self._arm_mode = 3
            self._armed = True
        elif arm_val == 1 and self._ins_nav_ready:
            if not self._armed:
                self.get_logger().info("ARM: normal arm (mode 0)")
                newly_armed = True
            self._arm_mode = 0
            self._armed = True

        # 复刻固件 SafetyMonitor::executeArm: 解锁瞬间把当前位姿设为 home offset
        # (roll/pitch 强制 0,核心内部将它作为控制原点/零姿态)。
        if newly_armed:
            self._set_home_offset_on_arm()

    def _set_home_offset_on_arm(self) -> None:
        """ARM 时把当前 map 位姿设为解锁原点,注入控制核 home offset。"""
        if not self.current_pose_ready:
            self.get_logger().warn("ARM: pose not ready, home offset skipped")
            return
        # self.pos 内部是 NED 度;核心期望弧度,roll/pitch 强制 0。
        pos6 = [
            self.pos.get('x', 0.0),
            self.pos.get('y', 0.0),
            self.pos.get('z', 0.0),
            0.0, 0.0,
            math.radians(self.pos.get('rz', 0.0)),
        ]
        try:
            self._core.set_home_offset(pos6)
            self.get_logger().info(
                f"Home offset set on ARM: x={pos6[0]:.2f} y={pos6[1]:.2f} "
                f"z={pos6[2]:.2f} yaw={math.degrees(pos6[5]):.1f}deg "
                f"-> current pose becomes origin (0,0,0,0,0,0)")
        except Exception as e:
            self.get_logger().error(f"set_home_offset failed: {e}")

    def _publish_zithbt(self) -> None:
        """~1Hz heartbeat, data = coarse ms tick (matches firmware; avoids hw_manager 7s watchdog)."""
        msg = UInt32()
        msg.data = int(time.monotonic() * 1000) & 0xFFFFFFFF
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
            # INS 重启 → 清除解锁原点(复刻固件 forceDisarmWithNeutralLevel)
            try:
                self._core.clear_home_offset()
            except Exception as e:
                self.get_logger().error(f"clear_home_offset failed: {e}")

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

        # Pose-ready gate for state publishing
        pos_world = [0.0] * 6
        vel_body = [0.0] * 6
        if self.current_pose_ready:
            # World NED position (radians for angular) — note pos stores deg internally
            pos_world = [
                self.pos['x'], self.pos['y'], self.pos['z'],
                math.radians(self.pos.get('rx', 0.0)),
                math.radians(self.pos.get('ry', 0.0)),
                math.radians(self.pos['rz']),
            ]
            vel_body = [
                self.vel['x'], self.vel['y'], self.vel['z'],
                self.vel['rx'], self.vel['ry'], self.vel['rz'],
            ]

            # Update INS alignment (1 → 5 → 4) using clock time
            self._update_ins_alignment()

            # Feed native core and step
            try:
                self._core.update_nav(pos_world, vel_body)
                forces = self._core.step()  # [Fx,Fy,Fz,Mroll,Mpitch,Myaw]
            except Exception as e:
                self.get_logger().error(f"core.step failed: {e}")
                forces = [0.0] * 6
        else:
            forces = [0.0] * 6

        with self._forces_lock:
            self.force_6dof = list(forces)
            self.thrust = self._mixer.mix6(*forces)

        # Keep control/actuation at 100Hz, but publish telemetry at rates that
        # are sufficient for navigation and task control.  Time-based gates
        # avoid the jitter of modulo counters when the control thread slips.
        publish_now = time.monotonic()
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

    # ── Sensor callbacks → estimator input / control state ─────────

    def _state_odom_cb(self, msg: PoseInfo) -> None:
        """Consume only the formal estimator state used by the control core."""
        values = (msg.robot_x, msg.robot_y, msg.robot_z, msg.robot_yaw)
        if not all(math.isfinite(float(value)) for value in values):
            return
        self.pos.update({
            'x': float(msg.robot_x), 'y': float(msg.robot_y),
            'z': float(msg.robot_z), 'rx': float(msg.robot_roll),
            'ry': float(msg.robot_pitch), 'rz': float(msg.robot_yaw),
        })
        self.current_pose_ready = True

    def _state_twist_cb(self, msg: TwistWithCovarianceStamped) -> None:
        """Consume only the formal estimator velocity used by the core."""
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        if not all(math.isfinite(float(value)) for value in values):
            return
        self.vel.update({
            'x': float(twist.linear.x), 'y': float(twist.linear.y),
            'z': float(twist.linear.z), 'rx': float(twist.angular.x),
            'ry': float(twist.angular.y), 'rz': float(twist.angular.z),
        })

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
        status.is_armed = self._armed
        status.arm_mode = self._arm_mode
        # control_level from native core (1=POS, 2=VEL, 3=ACTUATOR)
        status.control_level = self._core.control_level
        status.ins_state = self._ins_state
        status.navigation_ready = self._ins_nav_ready
        with self._forces_lock:
            f = list(self.force_6dof)
        status.forces = [f[0], f[1], f[2], f[3], f[4], f[5]]
        status.cycle_time_ms = 10.0
        status.battery_voltage = 16.8
        status.error_flags = 0
        self._publish_while_running(self.zit6_status_pub, status)
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
