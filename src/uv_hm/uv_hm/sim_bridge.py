"""Simulation bridge: emulates ZIT6 MCU firmware for Stonefish simulation.

Protocol matches real AUV: ZitSetpoint in → ZitStatus + Float32MultiArray out.
Cascade PID: pos → yaw-rate cascade, velocity inner loops, direct force mode.
Thruster mixer: xunyun 6-thruster geometry (direct NED body force → thrusts).
"""

import math
from dataclasses import dataclass

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, Image, Imu
from std_msgs.msg import Float64MultiArray, Float32, UInt8, Float32MultiArray

from stonefish_ros2.msg import DVL
from zit6_interfaces.msg import ZitSetpoint, ZitStatus


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


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
    def __init__(self) -> None:
        super().__init__("sim_bridge")

        # Internal state
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}     # NED deg
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}     # body, deg/s
        self.vel_world = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.thrust = [0.0] * 6
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.target_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.force_4dof = [0.0, 0.0, 0.0, 0.0]
        self._current_mode = 0  # 0=NONE, 1=POS, 2=VEL, 3=FORCE
        self.position_ctrl_enabled = False
        self.vel_ctrl_enabled = False
        self.current_pose_ready = False
        self.last_pid_time = self.get_clock().now()

        # === Publishers ===
        self.thruster_pub = self.create_publisher(
            Float64MultiArray, "/auv/thrusters_cmd", 10
        )
        self.zit6_status_pub = self.create_publisher(ZitStatus, "/zit6/state/status", 10)
        self.zit6_pos_pub = self.create_publisher(Float32MultiArray, "/zit6/state/pos", 10)
        self.zit6_vel_pub = self.create_publisher(Float32MultiArray, "/zit6/state/vel", 10)
        self.zit6_thr_pub = self.create_publisher(Float32MultiArray, "/zit6/state/thr", 10)

        # Camera republishers
        self.front_rect_left_pub = self.create_publisher(Image, "/auv/front_cam/left", 10)
        self.front_rect_right_pub = self.create_publisher(Image, "/auv/front_cam/right", 10)
        self.down_rect_left_pub = self.create_publisher(Image, "/auv/down_cam/left", 10)
        self.down_rect_right_pub = self.create_publisher(Image, "/auv/down_cam/right", 10)
        self.front_rect_pub = self.create_publisher(Image, "/auv/front_cam/stitched", 10)
        self.down_rect_pub = self.create_publisher(Image, "/auv/down_cam/stitched", 10)

        # Image stitching
        self.bridge = CvBridge()
        self.front_left_img = None
        self.front_right_img = None
        self.down_left_img = None
        self.down_right_img = None

        # === PID controllers ===
        # Position: body-frame error → body force
        self.pid_x = _Pid(800, 130.0, 200.0, i_limit=1000.0)
        self.pid_y = _Pid(600.0, 120.0, 150.0, i_limit=1000.0)
        self.pid_z = _Pid(600.0, 30.0, 800.0, i_limit=6200.0, z_thrust_ff=230.0)
        self.pid_yaw = _Pid(1.5, 1.0, 0.1, i_limit=30.0)
        self.pid_yaw_rate = _Pid(30.0, 20.0, 1.0, i_limit=600.0)

        # Velocity: body-frame velocity error → body force
        self.pid_vx = _Pid(300.0, 50.0, 10.0, i_limit=500.0)
        self.pid_vy = _Pid(300.0, 50.0, 10.0, i_limit=500.0)
        self.pid_vz = _Pid(300.0, 30.0, 50.0, i_limit=500.0, z_thrust_ff=230.0)
        self.pid_vyaw = _Pid(40.0, 20.0, 2.0, i_limit=300.0)

        # === Subscriptions ===
        self.create_subscription(ZitSetpoint, "/zit6/cmd/setpoint", self._setpoint_cb, 10)
        self.create_subscription(Float32, "/zit6/cmd/servo", self._servo_cb, 10)
        self.create_subscription(UInt8, "/zit6/cmd/light", self._light_cb, 10)
        self.create_subscription(Odometry, "/auv/odometry", self._odom_cb, 10)
        self.create_subscription(Imu, "/auv/imu", self._imu_cb, 10)
        self.create_subscription(DVL, "/auv/dvl", self._dvl_cb, 10)
        self.create_subscription(FluidPressure, "/auv/pressure", self._pressure_cb, 10)
        self.create_subscription(Image, "/sim/front_cam/left/image_color", self._front_left_img_cb, 10)
        self.create_subscription(Image, "/sim/front_cam/right/image_color", self._front_right_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/left/image_color", self._down_left_img_cb, 10)
        self.create_subscription(Image, "/sim/down_cam/right/image_color", self._down_right_img_cb, 10)

        self.create_timer(0.05, self._publish_state)
        self.create_timer(0.05, self._control_step)
        self.get_logger().info("sim_bridge started (ZIT6 protocol, xunyun mixer)")

    # ── ZIT6 setpoint callback ──────────────────────────────────────

    def _setpoint_cb(self, msg: ZitSetpoint) -> None:
        mode = msg.control_key & 0x03
        frame = msg.control_key & 0x10       # 0x10 = body
        incremental = msg.control_key & 0x20  # 0x20 = incremental

        if mode == 2:
            # Force mode: direct thrust, no PID
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = False
            self._current_mode = 3
            x = msg.x if (msg.type_mask & 0x01) else 0.0
            y = msg.y if (msg.type_mask & 0x02) else 0.0
            z = msg.z if (msg.type_mask & 0x04) else 0.0
            rz = msg.yaw if (msg.type_mask & 0x08) else 0.0
            self._publish_thrust_from_4dof(x, y, z, rz)

        elif mode == 1:
            # Velocity mode
            self.position_ctrl_enabled = False
            self.vel_ctrl_enabled = True
            self._current_mode = 2
            if msg.type_mask & 0x01:
                self.target_vel['x'] = msg.x
            if msg.type_mask & 0x02:
                self.target_vel['y'] = msg.y
            if msg.type_mask & 0x04:
                self.target_vel['z'] = msg.z
            if msg.type_mask & 0x08:
                self.target_vel['rz'] = math.degrees(msg.yaw)
            self.last_pid_time = self.get_clock().now()
            self.pid_vx.reset(); self.pid_vy.reset()
            self.pid_vz.reset(); self.pid_vyaw.reset()

        elif mode == 0:
            # Position mode
            self.vel_ctrl_enabled = False
            self._current_mode = 1
            yaw_deg = math.degrees(msg.yaw)

            if incremental:
                dx = msg.x if (msg.type_mask & 0x01) else 0.0
                dy = msg.y if (msg.type_mask & 0x02) else 0.0
                yaw_rad = math.radians(self.pos['rz'])
                cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                self.target_pos['x'] += cy * dx - sy * dy
                self.target_pos['y'] += sy * dx + cy * dy
                if msg.type_mask & 0x04:
                    self.target_pos['z'] += msg.z
                if msg.type_mask & 0x08:
                    self.target_pos['rz'] += yaw_deg
            else:
                if frame:
                    # Absolute body → world
                    yaw_rad = math.radians(self.pos['rz'])
                    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
                    if msg.type_mask & 0x01:
                        self.target_pos['x'] = self.pos['x'] + cy * msg.x - sy * msg.y
                    if msg.type_mask & 0x02:
                        self.target_pos['y'] = self.pos['y'] + sy * msg.x + cy * msg.y
                else:
                    # Absolute world
                    if msg.type_mask & 0x01:
                        self.target_pos['x'] = msg.x
                    if msg.type_mask & 0x02:
                        self.target_pos['y'] = msg.y
                if msg.type_mask & 0x04:
                    self.target_pos['z'] = msg.z
                if msg.type_mask & 0x08:
                    self.target_pos['rz'] = yaw_deg

            self.position_ctrl_enabled = True
            self.last_pid_time = self.get_clock().now()
            self.pid_x.reset(); self.pid_y.reset(); self.pid_z.reset()
            self.pid_yaw.reset(); self.pid_yaw_rate.reset()

    def _servo_cb(self, msg: Float32) -> None:
        pass

    def _light_cb(self, msg: UInt8) -> None:
        pass

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
        self.force_4dof = [x, y, z, rz]
        MAX_T = 1000.0

        # NED body force → 6 thrusters (xunyun geometry)
        h0 = (x + y + rz * 0.5)     # T0: aft-stbd diagonal
        h1 = (x - y - rz * 0.5)     # T1: aft-port diagonal
        h4 = -(x - y + rz * 0.5)    # T4: fwd-stbd diagonal
        h5 = -(x + y - rz * 0.5)    # T5: fwd-port diagonal
        h2 = z                        # HeaveBow
        h3 = z                        # HeaveStern

        raw = [h0, h1, h2, h3, h4, h5]
        for i in range(len(raw)):
            raw[i] = _clamp(raw[i] / MAX_T, -1.0, 1.0)

        cmd = Float64MultiArray()
        cmd.data = raw
        self.thruster_pub.publish(cmd)

        for i in range(6):
            self.thrust[i] = float(raw[i])

    # ── Control step ────────────────────────────────────────────────

    def _control_step(self) -> None:
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
        cmd_rz = -float(self.pid_yaw_rate.step(eyaw_rate, dt))

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
        _, _, yaw = self._quat_to_rpy(q.x, q.y, q.z, q.w)
        self.pos['rz'] = math.degrees(yaw)

        vx_w = msg.twist.twist.linear.x
        vy_w = msg.twist.twist.linear.y
        self.vel_world['x'] = vx_w
        self.vel_world['y'] = vy_w
        self.vel_world['z'] = msg.twist.twist.linear.z
        self.vel_world['rz'] = math.degrees(msg.twist.twist.angular.z)

        # World → body velocity rotation
        cy, sy = math.cos(yaw), math.sin(yaw)
        self.vel['x'] = vx_w * cy + vy_w * sy
        self.vel['y'] = -vx_w * sy + vy_w * cy
        self.vel['z'] = msg.twist.twist.linear.z
        self.vel['rz'] = self.vel_world['rz']

        self.current_pose_ready = True

    def _imu_cb(self, msg: Imu) -> None:
        self.vel['rz'] = math.degrees(msg.angular_velocity.z)

    def _dvl_cb(self, msg: DVL) -> None:
        self.vel['x'] = msg.velocity.x
        self.vel['y'] = msg.velocity.y
        self.vel['z'] = msg.velocity.z

    def _pressure_cb(self, msg: FluidPressure) -> None:
        pass

    # ── ZIT6 state publishing (20 Hz) ───────────────────────────────

    def _publish_state(self) -> None:
        status = ZitStatus()
        status.is_armed = True
        status.arm_mode = 3
        status.control_level = self._current_mode
        status.ins_state = 3
        status.navigation_ready = True
        status.forces = [float(self.force_4dof[i]) for i in range(4)]
        status.cycle_time_ms = 50.0
        status.battery_voltage = 16.8
        status.error_flags = 0
        self.zit6_status_pub.publish(status)

        pos_msg = Float32MultiArray()
        pos_msg.data = [self.pos['x'], self.pos['y'], self.pos['z'],
                        math.radians(self.pos['rz'])]
        self.zit6_pos_pub.publish(pos_msg)

        vel_msg = Float32MultiArray()
        vel_msg.data = [self.vel['x'], self.vel['y'], self.vel['z'],
                        math.radians(self.vel['rz'])]
        self.zit6_vel_pub.publish(vel_msg)

        thr_msg = Float32MultiArray()
        thr_msg.data = [float(self.force_4dof[i]) for i in range(4)]
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
