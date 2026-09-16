"""Simulation bridge: emulates ZIT6 MCU firmware for Stonefish simulation.

Protocol matches real AUV: ZitSetpoint in → ZitStatus + Float32MultiArray out.
Cascade PID: pos → yaw-rate cascade, velocity inner loops, direct force mode.
Thruster mixer: xunyun 6-thruster geometry (direct NED body force → thrusts).

**T5 note.**  The control maths, the PID configuration, the mixer and the frame
decoding no longer live here: they moved to ``uv_sim.mcu_control`` /
``uv_sim.mixer`` / ``uv_sim.pid_config`` / ``uv_sim.odometry``, and this node is
now the ROS glue around them.  The remaining split (one node per sensor, plus
``mcu_sim`` and ``nav_sim``) happens next; see ``docs/roadmap.md`` T5.

The module-level names re-exported below (``PID_PARAM_PATHS``, ``load_pid_params``,
``body_twist_from_odometry``, ...) are kept so that importers and tests written
against the pre-T5 layout keep working through the transition.
"""

from __future__ import annotations

import json
import math

import numpy as np
from cv_bridge import CvBridge
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

from uv_sim import runner
from uv_sim.mcu_control import McuController, quat_to_rpy
from uv_sim.mixer import MAX_THRUST_HIL, MAX_THRUST_SIL, mix_thruster_forces
from uv_sim.odometry import body_twist_from_odometry
from uv_sim.pid_config import (  # re-exported for the pre-T5 import surface
    DEFAULT_PID_PARAMS,
    PID_OPTIONAL_FIELDS,
    PID_PARAM_ATTRIBUTES,
    PID_PARAM_PATHS,
    PID_REQUIRED_FIELDS,
    load_pid_params,
    resolve_pid_config_path,
)

__all__ = [
    "SimBridgeNode",
    "body_twist_from_odometry",
    "main",
    "DEFAULT_PID_PARAMS",
    "PID_OPTIONAL_FIELDS",
    "PID_PARAM_ATTRIBUTES",
    "PID_PARAM_PATHS",
    "PID_REQUIRED_FIELDS",
    "load_pid_params",
    "resolve_pid_config_path",
]


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

        # The whole control state lives in the ROS-free core so it can be driven
        # deterministically from a unit test (see test_mcu_control.py).
        self._pid_params = load_pid_params(self._pid_config_path, self.get_logger())
        self._controller = McuController(
            self._pid_params, start_time=self._now_seconds()
        )
        # Per-channel attribute mirror (pid_x, pid_vx, ...) for the callers and
        # tests that address a controller by name.
        for path, attribute in PID_PARAM_ATTRIBUTES.items():
            setattr(self, attribute, self._controller.pids[path])

        if self._hil_mode:
            self._init_hil()
            self.get_logger().info("sim_bridge started in HIL mode (camera + thrust mixing)")
        else:
            self._init_full()
            self.get_logger().info("sim_bridge started (ZIT6 protocol, xunyun mixer)")
        self.get_logger().info(
            "PID gains loaded from "
            f"{self._pid_config_path or '<built-in defaults>'} "
            f"({len(self._controller.pids)} channels)"
        )

    # ── state owned by the controller (read-through views) ──────────────────

    @property
    def _pid_objects(self) -> dict:
        return self._controller.pids

    @property
    def pos(self) -> dict:
        return self._controller.pos

    @property
    def vel(self) -> dict:
        return self._controller.vel

    @property
    def force_4dof(self) -> list:
        return self._controller.force_4dof

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _init_full(self) -> None:
        """Full SIL mode: PID, thruster mixing, ZIT6 state machine + camera passthrough."""
        # Internal state
        self._tick = 0  # for rate-gated publishing

        self.thrust = [0.0] * 6

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
        roll, pitch, yaw = quat_to_rpy(q.x, q.y, q.z, q.w)
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
        immediate = self._controller.apply_setpoint(
            msg.control_key, msg.type_mask,
            msg.x, msg.y, msg.z, msg.yaw,
            self._now_seconds(),
        )
        if immediate is not None:      # force mode emits output straight away
            self._publish_thrust_from_4dof(*immediate)

    def _servo_cb(self, msg: Float32) -> None:
        self.get_logger().info(f"Servo: {msg.data:.3f} rad")

    def _light_cb(self, msg: UInt8) -> None:
        colors = {1: "red", 2: "yellow", 3: "green"}
        name = colors.get(msg.data, f"unknown ({msg.data})")
        self.get_logger().info(f"Light: {name}")

    # ── Heartbeat / ARM ─────────────────────────────────────────────

    def _agxhbt_cb(self, msg: UInt32) -> None:
        for message in self._controller.note_heartbeat(msg.data):
            self.get_logger().info(message)

    def _publish_zithbt(self) -> None:
        # Always armed — heartbeat timeout disabled
        msg = UInt32()
        msg.data = self._controller.next_heartbeat_seq()
        self.zit6_hbt_pub.publish(msg)

    # ── INS command ──────────────────────────────────────────────────

    def _ins_cb(self, msg: UInt8) -> None:
        for message in self._controller.ins_command(msg.data, self._now_seconds()):
            self.get_logger().info(message)

    def _update_ins_alignment(self) -> None:
        for message in self._controller.update_ins_alignment(self._now_seconds()):
            self.get_logger().info(message)

    # ── Parameter services ───────────────────────────────────────────

    def _get_params_cb(self, request, response):
        """Return PID parameters as JSON for requested paths."""
        response.success = True
        response.config_json = json.dumps(self._controller.get_params(list(request.paths)))
        return response

    def _update_params_cb(self, request, response):
        """Update PID parameters from JSON or path/value pairs."""
        try:
            if request.json:
                updates = json.loads(request.json)
            else:
                updates = dict(zip(request.paths, request.values))

            for message in self._controller.apply_param_updates(updates):
                self.get_logger().info(message)

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
        self._controller.force_4dof = [x, y, z, rz]
        # HIL mode: the MCU hands over forces already normalised to [-1, 1]
        max_thrust = MAX_THRUST_HIL if self._hil_mode else MAX_THRUST_SIL
        raw = mix_thruster_forces(x, y, z, rz, max_thrust=max_thrust)

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
        if not self._controller.current_pose_ready:
            return

        dt = self._controller.compute_dt(self._now_seconds())
        if self._controller.position_ctrl_enabled:
            self._position_pid_step(dt)
        elif self._controller.vel_ctrl_enabled:
            self._velocity_pid_step(dt)

    def _position_pid_step(self, dt: float) -> None:
        self._publish_thrust_from_4dof(*self._controller.step_position(dt))

    def _velocity_pid_step(self, dt: float) -> None:
        self._publish_thrust_from_4dof(*self._controller.step_velocity(dt))

    # ── Sensor callbacks ────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry) -> None:
        self._controller.apply_odometry(msg)

    def _imu_cb(self, msg: Imu) -> None:
        self._controller.apply_imu(msg)

    def _dvl_cb(self, msg: DVL) -> None:
        # B19: this overwrote the same vel fields as _odom_cb with a *different*
        # physical quantity (velocity at the DVL, i.e. origin + omega x r_dvl,
        # differing by 0.375*|omega_z|).  Kept here only until the split moves the
        # DVL value onto its own topic (/sim/sensor/dvl, nav_sim/dvl_sim split).
        self.vel['x'] = msg.velocity.x
        self.vel['y'] = msg.velocity.y
        self.vel['z'] = msg.velocity.z

    def _pressure_cb(self, msg: FluidPressure) -> None:
        pass

    # ── ZIT6 state publishing ───────────────────────────────────────
    # Published at different rates: status=10Hz, pos=30Hz, vel=60Hz, thr=30Hz

    def _publish_state(self) -> None:
        """Publish ZitStatus (10Hz)."""
        controller = self._controller
        status = ZitStatus()
        status.is_armed = controller.armed
        status.arm_mode = controller.arm_mode
        status.control_level = controller.current_mode
        status.ins_state = controller.ins_state
        status.navigation_ready = controller.ins_nav_ready
        status.forces = [controller.force_4dof[0], controller.force_4dof[1],
                         controller.force_4dof[2], 0.0, 0.0, controller.force_4dof[3]]
        status.cycle_time_ms = 50.0
        status.battery_voltage = 16.8
        status.error_flags = 0
        self.zit6_status_pub.publish(status)

    def _publish_pos(self) -> None:
        """Publish position (30Hz). 6-DOF: [x, y, z, roll_rad, pitch_rad, yaw_rad]."""
        pos = self._controller.pos
        pos_msg = Float32MultiArray()
        pos_msg.data = [
            pos['x'], pos['y'], pos['z'],
            math.radians(pos.get('rx', 0.0)),
            math.radians(pos.get('ry', 0.0)),
            math.radians(pos['rz']),
        ]
        self.zit6_pos_pub.publish(pos_msg)

    def _publish_vel(self) -> None:
        """Publish body velocity 6-DOF [vx, vy, vz, vroll_rad, vpitch_rad, vyaw_rad] (60Hz)."""
        vel = self._controller.vel
        vel_msg = Float32MultiArray()
        vel_msg.data = [
            vel['x'], vel['y'], vel['z'],
            math.radians(vel['rx']),
            math.radians(vel['ry']),
            math.radians(vel['rz']),
        ]
        self.zit6_vel_pub.publish(vel_msg)

    def _publish_thr(self) -> None:
        """Publish thrust 6-DOF [Fx, Fy, Fz, Mx(roll), My(pitch), Mz(yaw)] (30Hz)."""
        force = self._controller.force_4dof
        thr_msg = Float32MultiArray()
        # force_4dof = [Fx, Fy, Fz, Mz] → 6 元素, Mx/My 留空填 0
        thr_msg.data = [force[0], force[1], force[2], 0.0, 0.0, force[3]]
        self.zit6_thr_pub.publish(thr_msg)


def main(args=None) -> int:
    return runner.run(SimBridgeNode, args)


if __name__ == "__main__":
    raise SystemExit(main())
