"""Simulation bridge: emulates ZIT6 MCU firmware for Stonefish simulation.

Protocol matches real AUV: ZitSetpoint in → ZitStatus + Float32MultiArray out.
Cascade PID: pos → yaw-rate cascade, velocity inner loops, direct force mode.
Thruster mixer: xunyun 6-thruster geometry (direct NED body force → thrusts).
"""

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Image, Imu
from std_msgs.msg import Float64MultiArray, Float32, UInt8, UInt32, Float32MultiArray

try:
    from stonefish_ros2.msg import DVL
except ImportError:
    DVL = None
from zit6_interfaces.msg import ZitSetpoint, ZitStatus
from zit6_interfaces.srv import GetParams, UpdateParams


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def body_twist_from_odometry(msg: Odometry) -> tuple[float, float, float, float, float, float]:
    """Return ``(u, v, w, p, q, r)`` in the **body** frame from ``/auv/odometry``.

    ``nav_msgs/Odometry`` carries ``pose`` in ``header.frame_id`` (here
    ``world_ned``) but ``twist`` in ``child_frame_id``.  Stonefish honours that
    split by rotating the rigid-body velocity into the sensor frame before
    filling the message (``Odometry.cpp:59,65``:
    ``v = R_sensor<-world^T * v_point``); the odometry sensor sits at the
    body origin with ``rpy~0``, so the twist *is* the body velocity.

    Rotating it again by yaw would turn ``v_body`` into ``R(-psi) * v_body``
    (at psi=90 deg a pure surge ``u`` becomes ``(0, -u)``), which corrupted
    both ``/zit6/state/vel`` (SIL) and ``/zit6/sim/nav`` (HIL).  Keep this the
    single place where the convention is encoded.
    """
    twist = msg.twist.twist
    return (
        twist.linear.x,
        twist.linear.y,
        twist.linear.z,
        twist.angular.x,
        twist.angular.y,
        twist.angular.z,
    )


# ── PID gains: one external file is the single source of truth ──────────────
# The keys are the same paths the ZIT6 parameter service speaks, so the file,
# the service and the live controllers share one vocabulary.
PID_PARAM_PATHS = (
    'chassis.pid.pos.x',
    'chassis.pid.pos.y',
    'chassis.pid.pos.z',
    'chassis.pid.pos.yaw',
    'chassis.pid.pos.yaw_rate',   # inner loop of the yaw cascade
    'chassis.pid.vel.x',
    'chassis.pid.vel.y',
    'chassis.pid.vel.z',
    'chassis.pid.vel.yaw',
)

# path -> SimBridgeNode attribute holding the live _Pid instance
PID_PARAM_ATTRIBUTES = {
    'chassis.pid.pos.x': 'pid_x',
    'chassis.pid.pos.y': 'pid_y',
    'chassis.pid.pos.z': 'pid_z',
    'chassis.pid.pos.yaw': 'pid_yaw',
    'chassis.pid.pos.yaw_rate': 'pid_yaw_rate',
    'chassis.pid.vel.x': 'pid_vx',
    'chassis.pid.vel.y': 'pid_vy',
    'chassis.pid.vel.z': 'pid_vz',
    'chassis.pid.vel.yaw': 'pid_vyaw',
}

PID_REQUIRED_FIELDS = ('kp', 'ki', 'kd', 'i_limit')
PID_OPTIONAL_FIELDS = ('z_thrust_ff',)

# Used when the external file is missing or unreadable; keeps the SIL stack
# runnable instead of failing at startup.
# Last-resort defaults, kept in step with the shipped config/sim_pid.yaml.
# The file always wins when it is readable (see resolve_pid_config_path); these
# values only appear when no file is found at all, in which case the loader logs
# a warning.  They are a **mirror** of the last known tuning, not a second place
# to tune: edit sim_pid.yaml, then refresh these (or leave them slightly stale --
# only a completely missing file reaches them).
DEFAULT_PID_PARAMS = {
    'chassis.pid.pos.x': {'kp': 800.0, 'ki': 130.0, 'kd': 200.0, 'i_limit': 1000.0},
    'chassis.pid.pos.y': {'kp': 600.0, 'ki': 120.0, 'kd': 150.0, 'i_limit': 1000.0},
    'chassis.pid.pos.z': {'kp': 600.0, 'ki': 80.0, 'kd': 800.0, 'i_limit': 6200.0,
                          'z_thrust_ff': 0.0},
    'chassis.pid.pos.yaw': {'kp': 10.0, 'ki': 1.0, 'kd': 0.0, 'i_limit': 30.0},
    'chassis.pid.pos.yaw_rate': {'kp': 5.3, 'ki': 0.0, 'kd': 0.7, 'i_limit': 6.0},
    'chassis.pid.vel.x': {'kp': 600.0, 'ki': 50.0, 'kd': 10.0, 'i_limit': 500.0},
    'chassis.pid.vel.y': {'kp': 300.0, 'ki': 50.0, 'kd': 10.0, 'i_limit': 500.0},
    'chassis.pid.vel.z': {'kp': 300.0, 'ki': 30.0, 'kd': 50.0, 'i_limit': 500.0,
                          'z_thrust_ff': 0.0},
    'chassis.pid.vel.yaw': {'kp': 40.0, 'ki': 20.0, 'kd': 2.0, 'i_limit': 300.0},
}


def resolve_pid_config_path(explicit: str = "") -> Optional[Path]:
    """Locate ``sim_pid.yaml``: explicit parameter > source tree > package share.

    The **source tree wins** in a development checkout so that editing
    ``src/uv_sim/config/sim_pid.yaml`` takes effect on the next node start
    without a rebuild (the file installed under ``share/`` is a copy, not a
    symlink).  An installed deployment without the source tree falls back to
    the packaged copy.
    """
    if explicit:
        return Path(explicit).expanduser()

    source = Path(__file__).resolve().parent.parent / 'config' / 'sim_pid.yaml'
    if source.is_file():
        return source

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory('uv_sim')) / 'config' / 'sim_pid.yaml'
        if installed.is_file():
            return installed
    except Exception:  # ament index unavailable (e.g. bare pytest run)
        pass

    return None


def load_pid_params(path: Optional[Path], logger=None) -> dict:
    """Load PID gains from ``path``, falling back to ``DEFAULT_PID_PARAMS``.

    Every key in ``PID_PARAM_PATHS`` is always present in the result: the file
    overrides defaults, so a partial file is valid and unknown keys are only
    reported, never fatal.
    """

    def _warn(message: str) -> None:
        if logger is not None:
            logger.warning(message)

    params = {key: dict(DEFAULT_PID_PARAMS[key]) for key in PID_PARAM_PATHS}

    if path is None:
        _warn(
            "no sim_pid.yaml found (source tree, then package share); using the "
            "built-in PID defaults -- control gains may differ from the tuned file"
        )
        return params
    if not path.is_file():
        _warn(
            f"PID config {path} does not exist; using the built-in PID defaults -- "
            "control gains may differ from the tuned file"
        )
        return params

    try:
        raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    except Exception as exc:  # malformed YAML must not take the sim down
        _warn(f"failed to parse PID config {path}: {exc}; using built-in defaults")
        return params

    if not isinstance(raw, dict):
        _warn(f"PID config {path} must be a mapping of path -> gains; ignoring it")
        return params

    for key, gains in raw.items():
        if key not in params:
            _warn(f"PID config {path}: unknown channel '{key}' ignored")
            continue
        if not isinstance(gains, dict):
            _warn(f"PID config {path}: '{key}' must be a mapping; ignored")
            continue
        for field, value in gains.items():
            if field not in PID_REQUIRED_FIELDS + PID_OPTIONAL_FIELDS:
                _warn(f"PID config {path}: unknown field '{key}.{field}' ignored")
                continue
            try:
                params[key][field] = float(value)
            except (TypeError, ValueError):
                _warn(f"PID config {path}: '{key}.{field}'={value!r} is not a number; ignored")

    return params


@dataclass
class _Pid:
    kp: float
    ki: float
    kd: float
    i_limit: float = 0.5
    z_thrust_ff: float = 0.0

    def __post_init__(self) -> None:
        self.integral = 0.0
        self.prev_err = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self.prev_err = 0.0

    def step(self, err: float, dt: float) -> float:
        if dt <= 0.0:
            return self.kp * err + self.z_thrust_ff
        self.integral = _clamp(self.integral + err * dt, -self.i_limit, self.i_limit)
        derivative = (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integral + self.kd * derivative + self.z_thrust_ff


class SimBridgeNode(Node):
    def __init__(self, node_name: str = "sim_bridge", **kwargs) -> None:
        super().__init__(node_name, **kwargs)

        self.declare_parameter('hil_mode', False)
        self._hil_mode = self.get_parameter('hil_mode').value

        # PID gains live in one external file (see load_pid_params).  The
        # parameter lets an operator point at a tuning copy without touching
        # the installed package; empty means "use the packaged file".
        self.declare_parameter('pid_config_file', '')
        self._pid_config_path = resolve_pid_config_path(
            self.get_parameter('pid_config_file').value
        )

        if self._hil_mode:
            self._init_hil()
            self.get_logger().info("sim_bridge started in HIL mode (camera + thrust mixing)")
        else:
            self._init_full()
            self.get_logger().info("sim_bridge started (ZIT6 protocol, xunyun mixer)")

    def _init_full(self) -> None:
        """Full SIL mode: PID, thruster mixing, ZIT6 state machine + camera passthrough."""
        # Internal state
        self._tick = 0  # for rate-gated publishing

        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}     # NED deg
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rx': 0.0, 'ry': 0.0, 'rz': 0.0}  # body, deg/s
        self.thrust = [0.0] * 6
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.target_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.force_4dof = [0.0, 0.0, 0.0, 0.0]
        self._current_mode = 0  # 0=NONE, 1=POS, 2=VEL, 3=FORCE
        self.position_ctrl_enabled = False
        self.vel_ctrl_enabled = False
        self.current_pose_ready = False
        self.last_pid_time = self.get_clock().now()

        # ARM / heartbeat state (always armed, heartbeat is optional)
        self._armed = True
        self._arm_mode = 3
        self._last_heartbeat_time = self.get_clock().now()
        self._hb_seq = 0

        # INS state: 0=待机 1=粗对准 2=精对准 3=SINS/GPS/DVL 4=SINS/DVL 5=MRU
        self._ins_state = 1          # start in coarse alignment
        self._ins_nav_ready = False  # not ready until coarse done
        self._dvl_enabled = False    # DVL starts off
        self._ins_boot_time = self.get_clock().now()  # used for initial 1→5 transition
        self._ins_align_request = None  # time when SINS/DVL alignment was requested

        # === Publishers ===
        self.thruster_pub = self.create_publisher(
            Float64MultiArray, "/auv/thrusters_cmd", 10
        )
        self.zit6_status_pub = self.create_publisher(ZitStatus, "/zit6/state/status", 10)
        self.zit6_pos_pub = self.create_publisher(Float32MultiArray, "/zit6/state/pos", 10)
        self.zit6_vel_pub = self.create_publisher(Float32MultiArray, "/zit6/state/vel", 10)
        self.zit6_thr_pub = self.create_publisher(Float32MultiArray, "/zit6/state/thr", 10)
        self.zit6_hbt_pub = self.create_publisher(UInt32, "/zit6/state/zithbt", 10)

        # Camera republishers
        self._init_camera_publishers()

        # Image stitching
        self._init_camera_state()

        # === PID controllers ===
        # One external file is the single source of truth: the same dict backs
        # /zit6/get_params and builds the live _Pid objects, so the service can
        # never report gains that differ from the ones actually in use.
        #   file : <uv_sim share>/config/sim_pid.yaml
        #   param: pid_config_file:=/path/to/other.yaml
        # Structure: position mode = x/y/z single loops + yaw two-level cascade;
        # velocity mode = four single loops (details in the config file header).
        self._pid_params = load_pid_params(self._pid_config_path, self.get_logger())
        self._pid_objects = {}
        for path, attribute in PID_PARAM_ATTRIBUTES.items():
            gains = self._pid_params[path]
            pid = _Pid(
                gains['kp'],
                gains['ki'],
                gains['kd'],
                i_limit=gains['i_limit'],
                z_thrust_ff=gains.get('z_thrust_ff', 0.0),
            )
            setattr(self, attribute, pid)
            self._pid_objects[path] = pid
        self.get_logger().info(
            "PID gains loaded from "
            f"{self._pid_config_path or '<built-in defaults>'} "
            f"({len(self._pid_objects)} channels)"
        )

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

        # Camera subscriptions
        self._init_camera_subscriptions()

        self.create_timer(0.1, self._publish_zithbt)   # 10Hz heartbeat
        self.create_timer(0.0167, self._tick_60hz)      # 60Hz control + state

    def _init_hil(self) -> None:
        """HIL mode: camera passthrough + thrust mixing + nav aggregation for MCU."""
        self._init_camera_publishers()
        self._init_camera_state()
        self._init_camera_subscriptions()

        # Thrust mixing: MCU publishes 4-DOF forces → we mix to 6 thrusters
        self.thrust = [0.0] * 6
        self.force_4dof = [0.0, 0.0, 0.0, 0.0]
        self.thruster_pub = self.create_publisher(
            Float64MultiArray, "/auv/thrusters_cmd", 10
        )
        self.create_subscription(
            Float32MultiArray, "/zit6/state/thr", self._thrust_cb, 10
        )

        # Nav aggregation: subscribe Stonefish odometry + IMU, publish /zit6/sim/nav
        self._sim_pos = [0.0] * 6   # x, y, z, roll, pitch, yaw (NED)
        self._sim_vel = [0.0] * 6   # u, v, w, p, q, r (body FRD)
        self.sim_nav_pub = self.create_publisher(
            Float32MultiArray, "/zit6/sim/nav", 10
        )
        self.create_subscription(Odometry, "/auv/odometry", self._sim_nav_odom_cb, 10)
        self.create_subscription(Imu, "/auv/imu", self._sim_nav_imu_cb, 10)

    def _thrust_cb(self, msg: Float32MultiArray) -> None:
        """Receive 4-DOF forces from MCU, run thrust mixing, publish to Stonefish."""
        if len(msg.data) >= 4:
            self._publish_thrust_from_4dof(
                msg.data[0], msg.data[1], msg.data[2], msg.data[3]
            )

    def _sim_nav_odom_cb(self, msg: Odometry) -> None:
        """Aggregate Stonefish odometry into /zit6/sim/nav position + body velocity."""
        # Position (NED world frame)
        self._sim_pos[0] = msg.pose.pose.position.x
        self._sim_pos[1] = msg.pose.pose.position.y
        self._sim_pos[2] = msg.pose.pose.position.z
        # Orientation → roll, pitch, yaw
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self._sim_pos[3] = roll
        self._sim_pos[4] = pitch
        self._sim_pos[5] = yaw

        # Twist is already body-frame (see body_twist_from_odometry);
        # p, q, r are refreshed by the IMU callback.
        u, v, w, _, _, _ = body_twist_from_odometry(msg)
        self._sim_vel[0] = u  # u (surge)
        self._sim_vel[1] = v  # v (sway)
        self._sim_vel[2] = w  # w (heave)

        self._publish_sim_nav()

    def _sim_nav_imu_cb(self, msg: Imu) -> None:
        """Store angular velocity from IMU for /zit6/sim/nav."""
        self._sim_vel[3] = msg.angular_velocity.x  # p (roll rate)
        self._sim_vel[4] = msg.angular_velocity.y  # q (pitch rate)
        self._sim_vel[5] = msg.angular_velocity.z  # r (yaw rate)

    def _publish_sim_nav(self) -> None:
        """Publish aggregated nav data (12 floats) on /zit6/sim/nav."""
        nav = Float32MultiArray()
        nav.data = self._sim_pos + self._sim_vel
        self.sim_nav_pub.publish(nav)

    def _init_camera_publishers(self) -> None:
        self.front_rect_left_pub = self.create_publisher(Image, "/auv/front_cam/left", 10)
        self.front_rect_right_pub = self.create_publisher(Image, "/auv/front_cam/right", 10)
        self.down_rect_left_pub = self.create_publisher(Image, "/auv/down_cam/left", 10)
        self.down_rect_right_pub = self.create_publisher(Image, "/auv/down_cam/right", 10)
        self.front_rect_pub = self.create_publisher(Image, "/auv/front_cam/stitched", 10)
        self.down_rect_pub = self.create_publisher(Image, "/auv/down_cam/stitched", 10)

    def _init_camera_state(self) -> None:
        self.bridge = CvBridge()
        self.front_left_img = None
        self.front_right_img = None
        self.down_left_img = None
        self.down_right_img = None

    def _init_camera_subscriptions(self) -> None:
        self.create_subscription(Image, "/sim/front_cam/left/image_color", self._front_left_img_cb, 10)
        self.create_subscription(Image, "/sim/front_cam/right/image_color", self._front_right_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/left/image_color", self._down_left_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/right/image_color", self._down_right_img_cb, 10)

    # ── ZIT6 setpoint callback ──────────────────────────────────────

    def _setpoint_cb(self, msg: ZitSetpoint) -> None:
        mode = msg.control_key & 0x03
        frame = msg.control_key & 0x10       # 0x10 = body
        incremental = msg.control_key & 0x20  # 0x20 = incremental

        if mode == 2:
            # Force mode: bit=1 = keep current, bit=0 = apply
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = False
            self._current_mode = 3
            x = self.force_4dof[0] if (msg.type_mask & 0x01) else msg.x
            y = self.force_4dof[1] if (msg.type_mask & 0x02) else msg.y
            z = self.force_4dof[2] if (msg.type_mask & 0x04) else msg.z
            rz = self.force_4dof[3] if (msg.type_mask & 0x08) else msg.yaw
            self._publish_thrust_from_4dof(x, y, z, rz)

        elif mode == 1:
            # Velocity mode: bit=1 = ignore, bit=0 = apply
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = True
            self._current_mode = 2
            if not (msg.type_mask & 0x01):
                self.target_vel['x'] = msg.x
            if not (msg.type_mask & 0x02):
                self.target_vel['y'] = msg.y
            if not (msg.type_mask & 0x04):
                self.target_vel['z'] = msg.z
            if not (msg.type_mask & 0x08):
                self.target_vel['rz'] = math.degrees(msg.yaw)
            self.last_pid_time = self.get_clock().now()
            self.pid_vx.reset(); self.pid_vy.reset()
            self.pid_vz.reset(); self.pid_vyaw.reset()

        elif mode == 0:
            # Position mode: bit=1 = ignore, bit=0 = apply
            self.vel_ctrl_enabled = False
            self._current_mode = 1
            yaw_deg = math.degrees(msg.yaw)

            if incremental:
                dx = 0.0 if (msg.type_mask & 0x01) else msg.x
                dy = 0.0 if (msg.type_mask & 0x02) else msg.y
                yaw_rad = math.radians(self.pos['rz'])
                cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                self.target_pos['x'] += cy * dx - sy * dy
                self.target_pos['y'] += sy * dx + cy * dy
                if not (msg.type_mask & 0x04):
                    self.target_pos['z'] += msg.z
                if not (msg.type_mask & 0x08):
                    self.target_pos['rz'] += yaw_deg
            else:
                if frame:
                    # Absolute body → world
                    yaw_rad = math.radians(self.pos['rz'])
                    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                    if not (msg.type_mask & 0x01):
                        self.target_pos['x'] = self.pos['x'] + cy * msg.x - sy * msg.y
                    if not (msg.type_mask & 0x02):
                        self.target_pos['y'] = self.pos['y'] + sy * msg.x + cy * msg.y
                else:
                    # Absolute world
                    if not (msg.type_mask & 0x01):
                        self.target_pos['x'] = msg.x
                    if not (msg.type_mask & 0x02):
                        self.target_pos['y'] = msg.y
                if not (msg.type_mask & 0x04):
                    self.target_pos['z'] = msg.z
                if not (msg.type_mask & 0x08):
                    self.target_pos['rz'] = yaw_deg

            self.position_ctrl_enabled = True
            self.last_pid_time = self.get_clock().now()
            self.pid_x.reset(); self.pid_y.reset(); self.pid_z.reset()
            self.pid_yaw.reset(); self.pid_yaw_rate.reset()

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
        if arm_val == 3:
            # Force arm (bypass INS)
            self._arm_mode = 3
            if not self._armed:
                self.get_logger().info("ARM: force arm (mode 3)")
            self._armed = True
        elif arm_val == 1 and self._ins_nav_ready:
            # Normal arm (INS must be ready)
            self._arm_mode = 0
            if not self._armed:
                self.get_logger().info("ARM: normal arm (mode 0)")
            self._armed = True

    def _publish_zithbt(self) -> None:
        # Always armed — heartbeat timeout disabled

        msg = UInt32()
        self._hb_seq += 1
        msg.data = self._hb_seq
        self.zit6_hbt_pub.publish(msg)

    # ── INS command ──────────────────────────────────────────────────

    def _ins_cb(self, msg: UInt8) -> None:
        cmd = msg.data
        if cmd == 1:
            self._dvl_enabled = True
            self.get_logger().info("INS: DVL enabled, requesting SINS/DVL alignment")
            if self._ins_state == 5:
                self._ins_align_request = self.get_clock().now()
            elif self._ins_state == 1:
                self.get_logger().info("INS: waiting for coarse alignment first")
        elif cmd == 2:
            self._dvl_enabled = False
            self._ins_align_request = None
            self.get_logger().info("INS: DVL disabled, fallback to MRU")
            if self._ins_state == 4:
                self._ins_state = 5
        elif cmd == 3:
            self.get_logger().info("INS: restarting alignment...")
            self._ins_state = 1
            self._ins_nav_ready = False
            self._dvl_enabled = False
            self._ins_align_request = None
            self._ins_boot_time = self.get_clock().now()

    def _update_ins_alignment(self) -> None:
        """INS state machine: 1→5→4 (or 1→5 if no DVL)."""
        now = self.get_clock().now()

        # Phase 1: coarse alignment (1 → 5, ~1.5s after boot)
        if self._ins_state == 1:
            elapsed = (now - self._ins_boot_time).nanoseconds / 1e9
            if elapsed >= 1.5:
                self._ins_state = 5
                self._ins_nav_ready = True
                self.get_logger().info("INS: coarse alignment done → MRU mode")
                # If DVL was already requested while in state 1, start alignment now
                if self._dvl_enabled:
                    self._ins_align_request = now

        # Phase 2: SINS/DVL alignment (5 → 4, ~1.5s after request)
        if self._ins_state == 5 and self._ins_align_request is not None:
            elapsed = (now - self._ins_align_request).nanoseconds / 1e9
            if elapsed >= 1.5:
                self._ins_state = 4
                self._ins_align_request = None
                self.get_logger().info("INS: SINS/DVL alignment complete")

    # ── Parameter services ───────────────────────────────────────────

    def _get_params_cb(self, request, response):
        """Return PID parameters as JSON for requested paths."""
        if not request.paths:
            # Return full config
            response.success = True
            response.config_json = json.dumps(self._pid_params)
            return response

        result = {}
        for path in request.paths:
            value = self._pid_params.get(path)
            if value is not None:
                result[path] = value

        response.success = True
        response.config_json = json.dumps(result)
        return response

    def _update_params_cb(self, request, response):
        """Update PID parameters from JSON or path/value pairs."""
        try:
            if request.json:
                updates = json.loads(request.json)
            else:
                updates = dict(zip(request.paths, request.values))

            for path, value in updates.items():
                if path in self._pid_params:
                    gains = self._pid_params[path]
                    if isinstance(value, dict):
                        # Full gain dict: {"kp": 800, "ki": 130.0, ...}
                        for k, v in value.items():
                            if k in gains:
                                gains[k] = float(v)
                    elif isinstance(value, (int, float)):
                        # Single key: interpret as kp
                        gains['kp'] = float(value)

                    # Write-through to live PID object
                    pid = self._pid_objects.get(path)
                    if pid is not None:
                        pid.kp = gains['kp']
                        pid.ki = gains.get('ki', 0.0)
                        pid.kd = gains.get('kd', 0.0)
                        pid.i_limit = gains.get('i_limit', 0.5)
                        if 'z_thrust_ff' in gains:
                            pid.z_thrust_ff = gains['z_thrust_ff']
                    self.get_logger().info(f"Params: {path} → {gains}")

            response.success = True
            response.message = "Parameters updated"
        except Exception as e:
            response.success = False
            response.message = str(e)

        return response

    # ── Image callbacks ─────────────────────────────────────────────

    def _front_left_img_cb(self, msg: Image) -> None:
        self.front_rect_left_pub.publish(msg)
        self.front_left_img = msg
        self._publish_stitched_front()

    def _front_right_img_cb(self, msg: Image) -> None:
        self.front_rect_right_pub.publish(msg)
        self.front_right_img = msg
        self._publish_stitched_front()

    def _down_left_img_cb(self, msg: Image) -> None:
        self.down_rect_left_pub.publish(msg)
        self.down_left_img = msg
        self._publish_stitched_down()

    def _down_right_img_cb(self, msg: Image) -> None:
        self.down_rect_right_pub.publish(msg)
        self.down_right_img = msg
        self._publish_stitched_down()

    def _publish_stitched_front(self) -> None:
        if self.front_left_img and self.front_right_img:
            try:
                left = self.bridge.imgmsg_to_cv2(self.front_left_img, "bgr8")
                right = self.bridge.imgmsg_to_cv2(self.front_right_img, "bgr8")
                stitched = np.hstack((left, right))
                out = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                out.header = self.front_left_img.header
                self.front_rect_pub.publish(out)
            except Exception as e:
                self.get_logger().error(f"Front stitch failed: {e}")

    def _publish_stitched_down(self) -> None:
        if self.down_left_img and self.down_right_img:
            try:
                left = self.bridge.imgmsg_to_cv2(self.down_left_img, "bgr8")
                right = self.bridge.imgmsg_to_cv2(self.down_right_img, "bgr8")
                stitched = np.hstack((left, right))
                out = self.bridge.cv2_to_imgmsg(stitched, "bgr8")
                out.header = self.down_left_img.header
                self.down_rect_pub.publish(out)
            except Exception as e:
                self.get_logger().error(f"Down stitch failed: {e}")

    # ── Thruster mixer (xunyun 6-thruster geometry) ────────────────

    def _publish_thrust_from_4dof(self, x: float, y: float, z: float, rz: float) -> None:
        # DISABLED: always armed for now
        # if not self._armed:
        #     x = y = z = rz = 0.0
        self.force_4dof = [x, y, z, rz]
        MAX_T = 1000.0
        if self._hil_mode:
            MAX_T = 1.0  # HIL mode: forces are normalized to [-1, 1]

        # NED body force → 6 thrusters (xunyun geometry)
        h0 = (x + y - rz )     # T0: aft-stbd diagonal
        h1 = (x - y + rz )     # T1: aft-port diagonal
        h4 = -(x - y - rz )    # T4: fwd-stbd diagonal
        h5 = -(x + y + rz )    # T5: fwd-port diagonal
        h2 = z*0.8;                        # HeaveBow
        h3 = z*0.8;                        # HeaveStern

        raw = [h0, h1, h2, h3, h4, h5]
        for i in range(len(raw)):
            raw[i] = _clamp(raw[i] / MAX_T, -1.0, 1.0)

        cmd = Float64MultiArray()
        cmd.data = raw
        self.thruster_pub.publish(cmd)

        for i in range(6):
            self.thrust[i] = float(raw[i])

    # ── Control step ────────────────────────────────────────────────

    def _tick_60hz(self) -> None:
        """60Hz control step + rate-gated state publishing."""
        self._tick = (self._tick + 1) % 60

        # State publishing (gate by tick count)
        #   status: 10Hz (every 6 ticks, tick 0)
        #   pos:    30Hz (every 2 ticks, tick 0,2,4...)
        #   vel:    60Hz (every tick)
        #   thr:    30Hz (every 2 ticks, tick 0,2,4...)
        if self._tick % 6 == 0:
            self._publish_state()
        if self._tick % 2 == 0:
            self._publish_pos()
            self._publish_thr()
        self._publish_vel()

        # Control step
        self._update_ins_alignment()
        if not self.current_pose_ready:
            return

        now = self.get_clock().now()
        dt = (now - self.last_pid_time).nanoseconds / 1e9
        self.last_pid_time = now
        dt = _clamp(dt, 0.001, 0.2)

        if self.position_ctrl_enabled:
            self._position_pid_step(dt)
        elif self.vel_ctrl_enabled:
            self._velocity_pid_step(dt)

    def _position_pid_step(self, dt: float) -> None:
        x, y, z, yaw = self.pos['x'], self.pos['y'], self.pos['z'], self.pos['rz']

        ex_w = self.target_pos['x'] - x
        ey_w = self.target_pos['y'] - y

        yaw_rad = math.radians(yaw)
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        ex_body = cy * ex_w + sy * ey_w
        ey_body = -sy * ex_w + cy * ey_w

        ez = self.target_pos['z'] - z
        eyaw = self._wrap_angle_deg(self.target_pos['rz'] - yaw)

        target_yaw_rate = self.pid_yaw.step(eyaw, dt)
        target_yaw_rate = _clamp(target_yaw_rate, -45.0, 45.0)
        eyaw_rate = target_yaw_rate - self.vel['rz']

        cmd_x = float(self.pid_x.step(ex_body, dt))
        cmd_y = float(self.pid_y.step(ey_body, dt))
        cmd_z = float(self.pid_z.step(ez, dt))
        cmd_rz = float(self.pid_yaw_rate.step(eyaw_rate, dt))

        self._publish_thrust_from_4dof(cmd_x, cmd_y, cmd_z, cmd_rz)

    def _velocity_pid_step(self, dt: float) -> None:
        evx = self.target_vel['x'] - self.vel['x']
        evy = self.target_vel['y'] - self.vel['y']
        evz = self.target_vel['z'] - self.vel['z']
        evyaw = self.target_vel['rz'] - self.vel['rz']

        cmd_x = float(self.pid_vx.step(evx, dt))
        cmd_y = float(self.pid_vy.step(evy, dt))
        cmd_z = float(self.pid_vz.step(evz, dt))
        cmd_rz = float(self.pid_vyaw.step(evyaw, dt))

        self._publish_thrust_from_4dof(cmd_x, cmd_y, cmd_z, cmd_rz)

    # ── Sensor callbacks ────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self.pos['x'] = msg.pose.pose.position.x
        self.pos['y'] = msg.pose.pose.position.y
        self.pos['z'] = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
        roll, pitch, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self.pos['rx'] = math.degrees(roll)
        self.pos['ry'] = math.degrees(pitch)
        self.pos['rz'] = math.degrees(yaw)

        # Twist is already body-frame -- no yaw rotation here (see
        # body_twist_from_odometry).  The DVL callback overwrites x/y/z with its
        # own measurement, so this path only fills the gaps between DVL samples.
        u, v, w, p, q_rate, r = body_twist_from_odometry(msg)
        self.vel['x'] = u
        self.vel['y'] = v
        self.vel['z'] = w
        self.vel['rx'] = math.degrees(p)
        self.vel['ry'] = math.degrees(q_rate)
        self.vel['rz'] = math.degrees(r)

        self.current_pose_ready = True

    def _imu_cb(self, msg: Imu) -> None:
        self.vel['rx'] = math.degrees(msg.angular_velocity.x)
        self.vel['ry'] = math.degrees(msg.angular_velocity.y)
        self.vel['rz'] = math.degrees(msg.angular_velocity.z)

    def _dvl_cb(self, msg: DVL) -> None:
        self.vel['x'] = msg.velocity.x
        self.vel['y'] = msg.velocity.y
        self.vel['z'] = msg.velocity.z

    def _pressure_cb(self, msg: FluidPressure) -> None:
        pass

    # ── ZIT6 state publishing ───────────────────────────────────────
    # Published at different rates: status=10Hz, pos=30Hz, vel=60Hz, thr=30Hz

    def _publish_state(self) -> None:
        """Publish ZitStatus (10Hz)."""
        status = ZitStatus()
        status.is_armed = self._armed
        status.arm_mode = self._arm_mode
        status.control_level = self._current_mode
        status.ins_state = self._ins_state
        status.navigation_ready = self._ins_nav_ready
        status.forces = [self.force_4dof[0], self.force_4dof[1], self.force_4dof[2],
                          0.0, 0.0, self.force_4dof[3]]
        status.cycle_time_ms = 50.0
        status.battery_voltage = 16.8
        status.error_flags = 0
        self.zit6_status_pub.publish(status)

    def _publish_pos(self) -> None:
        """Publish position (30Hz). 6-DOF: [x, y, z, roll_rad, pitch_rad, yaw_rad]."""
        pos_msg = Float32MultiArray()
        pos_msg.data = [
            self.pos['x'], self.pos['y'], self.pos['z'],
            math.radians(self.pos.get('rx', 0.0)),
            math.radians(self.pos.get('ry', 0.0)),
            math.radians(self.pos['rz']),
        ]
        self.zit6_pos_pub.publish(pos_msg)

    def _publish_vel(self) -> None:
        """Publish body velocity 6-DOF [vx, vy, vz, vroll_rad, vpitch_rad, vyaw_rad] (60Hz)."""
        vel_msg = Float32MultiArray()
        vel_msg.data = [
            self.vel['x'], self.vel['y'], self.vel['z'],
            math.radians(self.vel['rx']),
            math.radians(self.vel['ry']),
            math.radians(self.vel['rz']),
        ]
        self.zit6_vel_pub.publish(vel_msg)

    def _publish_thr(self) -> None:
        """Publish thrust 6-DOF [Fx, Fy, Fz, Mx(roll), My(pitch), Mz(yaw)] (30Hz)."""
        thr_msg = Float32MultiArray()
        # force_4dof = [Fx, Fy, Fz, Mz] → 6 元素, Mx/My 留空填 0
        thr_msg.data = [
            self.force_4dof[0], self.force_4dof[1], self.force_4dof[2],
            0.0, 0.0, self.force_4dof[3],
        ]
        self.zit6_thr_pub.publish(thr_msg)

    # ── Utilities ───────────────────────────────────────────────────

    @staticmethod
    def _quat_to_rpy(x: float, y: float, z: float, w: float):
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        pitch = math.asin(_clamp(sinp, -1.0, 1.0))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    @staticmethod
    def _wrap_angle_deg(angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
