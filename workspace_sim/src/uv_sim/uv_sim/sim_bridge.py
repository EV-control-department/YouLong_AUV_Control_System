"""Simulation bridge: emulate the ZIT6 MCU firmware, driven by the native C++ control core.

Protocol matches real AUV: ZitSetpoint in → ZitStatus + Float32MultiArray out.
Unlike the old homegrown Python cascade PID, the 100Hz control loop runs in the
native ZIT6 control core (zit6_control_core.Zit6Controller) — a host compile of
the actual firmware cascade controller.
Thruster mixing (xunyun 6-thruster geometry) happens here in Python, since the
real firmware leaves that to the external motor controller board.
"""

import json
import math
import threading
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Image, Imu
from std_msgs.msg import Float64MultiArray, Float32, UInt8, UInt32, Float32MultiArray

try:
    from stonefish_ros2.msg import DVL
except ImportError:
    DVL = None
from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from zit6_interfaces.srv import GetParams, UpdateParams

import zit6_control_core
from zit6_control_core import Zit6Controller

from .camera_passthrough import CameraPassthrough
from .coordinate_convention import scene_to_odom_ned, scene_yaw_to_odom_ned
from .thrust_mixer import ThrustMixer


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

    candidates = [
        Path("/home/doc049/dev/UUV/YouLong_AUV_Control_System/workspace_sim/src/zit6_control_core/sim_config.json"),
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

        # The position reported by Stonefish is in the scene/world frame.
        # Keep the spawn position as the simulation ``pos`` origin so the
        # simulated vehicle has the same local reference after every launch.
        # Only translation is removed; the initial attitude remains in pos.
        self._position_origin = None
        self._position_origin_lock = threading.Lock()

        self.declare_parameter('hil_mode', False)
        self._hil_mode = self.get_parameter('hil_mode').value
        self.declare_parameter('camera_stitch_fps', 10.0)
        self.declare_parameter('publish_raw_camera_topics', False)

        self.cam = CameraPassthrough(
            self.get_parameter('camera_stitch_fps').value,
            publish_raw_views=_as_bool(
                self.get_parameter('publish_raw_camera_topics').value))
        self.cam.bind(self)

        if self._hil_mode:
            self._init_hil()
            self.get_logger().info("sim_bridge started in HIL mode (thrust mixing + nav feed)")
        else:
            self._init_full()
            self.get_logger().info("sim_bridge started (native ZIT6 core, xunyun mixer)")

    # ── Full SIL mode: native ZIT6 control core + mixing ────────────

    def _init_full(self) -> None:
        self._tick = 0
        self._last_status_publish_s = float('-inf')
        self._last_telemetry_publish_s = float('-inf')

        # Internal state (host policy, mirrors firmware MicroRosPublisher semantics)
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}      # NED deg (internal convenience)
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}  # body, rad/s
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
        self._core = Zit6Controller(chassis) if chassis else Zit6Controller({})
        self.get_logger().info(
            f"ZIT6 native core loaded, control_level="
            f"{self._core.control_level}")

        # === Publishers ===
        self.thruster_pub = self.create_publisher(Float64MultiArray, "/auv/thrusters_cmd", 10)
        self.zit6_status_pub = self.create_publisher(ZitStatus, "/zit6/state/status", 10)
        self.zit6_pos_pub = self.create_publisher(Float32MultiArray, "/zit6/state/pos", 10)
        self.zit6_vel_pub = self.create_publisher(Float32MultiArray, "/zit6/state/vel", 10)
        self.zit6_thr_pub = self.create_publisher(Float32MultiArray, "/zit6/state/thr", 10)
        self.zit6_hbt_pub = self.create_publisher(UInt32, "/zit6/state/zithbt", 10)

        # === Subscriptions ===
        self.create_subscription(ZitSetpoint, "/zit6/cmd/setpoint", self._setpoint_cb, 10)
        self.create_subscription(Float32, "/zit6/cmd/servo", self._servo_cb, 10)
        self.create_subscription(UInt8, "/zit6/cmd/light", self._light_cb, 10)
        self.create_subscription(Odometry, "/auv/odometry", self._odom_cb, 10)
        self.create_subscription(Imu, "/auv/imu", self._imu_cb, 10)
        if DVL is not None:
            self.create_subscription(DVL, "/auv/dvl", self._dvl_cb, 10)
        self.create_subscription(FluidPressure, "/auv/pressure", self._pressure_cb, 10)
        self.create_subscription(UInt32, "/zit6/cmd/agxhbt", self._agxhbt_cb, 10)
        self.create_subscription(UInt8, "/zit6/cmd/ins", self._ins_cb, 10)

        # === Services ===
        self.create_service(GetParams, "/zit6/get_params", self._get_params_cb)
        self.create_service(UpdateParams, "/zit6/update_params", self._update_params_cb)

        self._mixer = ThrustMixer(heave_factor=0.8)

        # zithbt ~1Hz (firmware publishes ms-tick ~1Hz; hw_manager watchdog 7s)
        self.create_timer(1.0, self._publish_zithbt)

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
        self.thruster_pub = self.create_publisher(Float64MultiArray, "/auv/thrusters_cmd", 10)
        self.create_subscription(Float32MultiArray, "/zit6/state/thr", self._thrust_cb, 10)
        self._mixer = ThrustMixer()

        # Nav aggregation for MCU
        self._sim_pos = [0.0] * 6
        self._sim_vel = [0.0] * 6
        self.sim_nav_pub = self.create_publisher(Float32MultiArray, "/zit6/sim/nav", 10)
        self.create_subscription(Odometry, "/auv/odometry", self._sim_nav_odom_cb, 10)
        self.create_subscription(Imu, "/auv/imu", self._sim_nav_imu_cb, 10)

    def _thrust_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) >= 6:
            self._publish_thrust_from_6dof(*msg.data[:6])

    def _sim_nav_odom_cb(self, msg: Odometry) -> None:
        self._sim_pos[0], self._sim_pos[1], self._sim_pos[2] = self._relative_position(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        q = msg.pose.pose.orientation
        roll, pitch, scene_yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        yaw = scene_yaw_to_odom_ned(scene_yaw)
        self._sim_pos[3], self._sim_pos[4], self._sim_pos[5] = roll, pitch, yaw
        vx_w, vy_w, _ = scene_to_odom_ned(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, 0.0)
        cy, sy = math.cos(yaw), math.sin(yaw)
        self._sim_vel[0] = vx_w * cy + vy_w * sy
        self._sim_vel[1] = -vx_w * sy + vy_w * cy
        self._sim_vel[2] = msg.twist.twist.linear.z
        self._publish_sim_nav()

    def _sim_nav_imu_cb(self, msg: Imu) -> None:
        self._sim_vel[3] = msg.angular_velocity.x
        self._sim_vel[4] = msg.angular_velocity.y
        self._sim_vel[5] = msg.angular_velocity.z
        self._publish_sim_nav()

    def _publish_sim_nav(self) -> None:
        nav = Float32MultiArray()
        nav.data = self._sim_pos + self._sim_vel
        self.sim_nav_pub.publish(nav)

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
        self.zit6_hbt_pub.publish(msg)

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
        last = time.monotonic()
        while rclpy.ok() and not self._control_stop.is_set():
            now = time.monotonic()
            elapsed = now - last
            if elapsed >= 0.01:  # ≥10ms guard, mirror firmware kDt
                self._control_tick()
                last = now
            time.sleep(0.001)

    def _control_tick(self) -> None:
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
        if publish_now - self._last_telemetry_publish_s >= (1.0 / 30.0):
            self._publish_pos(pos_world)
            self._publish_vel(vel_body)
            self._publish_thr()
            self._last_telemetry_publish_s = publish_now

        # Thrust to Stonefish
        cmd = Float64MultiArray()
        cmd.data = self.thrust
        self.thruster_pub.publish(cmd)

    def _publish_thrust_from_6dof(self, fx, fy, fz, mroll, mpitch, myaw) -> None:
        """Mixin (HIL): forces come from real MCU via /zit6/state/thr."""
        with self._forces_lock:
            self.force_6dof = [fx, fy, fz, mroll, mpitch, myaw]
            self.thrust = self._mixer.mix6(fx, fy, fz, mroll, mpitch, myaw)
        cmd = Float64MultiArray()
        cmd.data = self.thrust
        self.thruster_pub.publish(cmd)

    # ── Sensor callbacks → NavState ────────────────────────────────

    def _relative_position(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Return Stonefish position relative to the AUV spawn position.

        The first odometry sample is the spawn pose for the simulation.  The
        origin is shared by SIL and HIL so ``/zit6/state/pos`` and
        ``/zit6/sim/nav`` expose the same local position convention.
        """
        with self._position_origin_lock:
            if self._position_origin is None:
                self._position_origin = (float(x), float(y), float(z))
                self.get_logger().info(
                    'Simulation pos origin set to Stonefish spawn: '
                    f'x={x:.3f}, y={y:.3f}, z={z:.3f}')
            ox, oy, oz = self._position_origin
        return scene_to_odom_ned(float(x) - ox, float(y) - oy, float(z) - oz)

    def _odom_cb(self, msg: Odometry) -> None:
        self.pos['x'], self.pos['y'], self.pos['z'] = self._relative_position(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )
        q = msg.pose.pose.orientation
        roll, pitch, scene_yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        yaw = scene_yaw_to_odom_ned(scene_yaw)
        self.pos['rx'] = math.degrees(roll)
        self.pos['ry'] = math.degrees(pitch)
        self.pos['rz'] = math.degrees(yaw)

        vx_w, vy_w, vz_w = scene_to_odom_ned(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        )
        self.vel_world['x'], self.vel_world['y'], self.vel_world['z'] = (
            vx_w, vy_w, vz_w)
        self.vel_world['rx'] = msg.twist.twist.angular.x
        self.vel_world['ry'] = msg.twist.twist.angular.y
        self.vel_world['rz'] = msg.twist.twist.angular.z

        # World → body linear velocity (yaw rotation, NED FRD)
        cy, sy = math.cos(yaw), math.sin(yaw)
        self.vel['x'] = vx_w * cy + vy_w * sy
        self.vel['y'] = -vx_w * sy + vy_w * cy
        self.vel['z'] = vz_w
        self.vel['rx'] = self.vel_world['rx']
        self.vel['ry'] = self.vel_world['ry']
        self.vel['rz'] = self.vel_world['rz']

        self.current_pose_ready = True

    def _imu_cb(self, msg: Imu) -> None:
        self.vel['rx'] = msg.angular_velocity.x
        self.vel['ry'] = msg.angular_velocity.y
        self.vel['rz'] = msg.angular_velocity.z

    def _dvl_cb(self, msg: DVL) -> None:
        self.vel['x'] = msg.velocity.x
        self.vel['y'] = msg.velocity.y
        self.vel['z'] = msg.velocity.z

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
        self.zit6_status_pub.publish(status)

    def _publish_pos(self, pos_world) -> None:
        pos_msg = Float32MultiArray()
        pos_msg.data = pos_world  # [x,y,z,roll_rad,pitch_rad,yaw_rad]
        self.zit6_pos_pub.publish(pos_msg)

    def _publish_vel(self, vel_body) -> None:
        vel_msg = Float32MultiArray()
        vel_msg.data = vel_body  # body [u,v,w,p,q,r]
        self.zit6_vel_pub.publish(vel_msg)

    def _publish_thr(self) -> None:
        thr_msg = Float32MultiArray()
        with self._forces_lock:
            thr_msg.data = list(self.force_6dof)
        self.zit6_thr_pub.publish(thr_msg)

    # ── Utilities ───────────────────────────────────────────────────

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

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
        node.destroy_node()
        # launch may already have shut down the default context while
        # delivering SIGINT.  try_shutdown keeps a normal Ctrl-C from
        # becoming an exit-code-1 failure.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
