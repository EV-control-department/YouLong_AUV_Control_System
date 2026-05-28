"""Simulation bridge node: subscribes to Stonefish sensors, runs cascade PID, publishes state.

This node simulates the MCU firmware behavior for the simulation environment.
It implements a cascade PID control algorithm:
  XY plane: position error -> magnitude+angle -> speed PID -> decompose vx/vy -> velocity PIDs -> force
  Z axis:   position PID -> force (with buoyancy feedforward)
  Yaw axis: position PID -> yaw rate PID -> torque
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray, Float32MultiArray
from sensor_msgs.msg import Imu, FluidPressure, Image
from nav_msgs.msg import Odometry

try:
    from stonefish_ros2.msg import DVL
except ImportError:
    DVL = None

from uv_msgs.msg import MotionCommand, AuvState, PidGains, PidState

from uv_hm.pid_controller import PidController, PidGains as _PidGains
from uv_hm.thrust_mixer import ThrustMixer


def _wrap_angle_deg(angle: float) -> float:
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


def _wrap_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw in radians (NED frame)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CascadePidController:
    """Cascade PID controller for AUV 4-DOF control.

    XY plane: position error (world) -> magnitude + angle -> speed PID
              -> decompose to vx/vy -> velocity PIDs -> force
    Z axis:   position PID -> force (with buoyancy feedforward)
    Yaw axis: position PID -> yaw rate PID -> torque
    """

    def __init__(self):
        # Outer loop: error magnitude -> speed magnitude
        self.pid_speed_mag = PidController(_PidGains(
            kp=0.5, ki=0.05, kd=0.1, i_limit=2.0, output_limit=2.0
        ))

        # Inner loop: body-frame velocity
        self.pid_vx = PidController(_PidGains(
            kp=300.0, ki=50.0, kd=10.0, i_limit=500.0, output_limit=1000.0
        ))
        self.pid_vy = PidController(_PidGains(
            kp=300.0, ki=50.0, kd=10.0, i_limit=500.0, output_limit=1000.0
        ))

        # Z axis: position PID with buoyancy feedforward
        self.pid_z = PidController(_PidGains(
            kp=600.0, ki=30.0, kd=800.0, i_limit=6200.0,
            output_limit=1000.0, feedforward=230.0
        ))

        # Yaw: position -> rate cascade
        self.pid_yaw = PidController(_PidGains(
            kp=1.5, ki=1.0, kd=0.1, i_limit=30.0, output_limit=45.0
        ))
        self.pid_yaw_rate = PidController(_PidGains(
            kp=30.0, ki=20.0, kd=1.0, i_limit=600.0, output_limit=1000.0
        ))

        # Velocity-only PIDs (for velocity control mode)
        self.pid_vz = PidController(_PidGains(
            kp=300.0, ki=30.0, kd=50.0, i_limit=500.0,
            output_limit=1000.0, feedforward=230.0
        ))
        self.pid_vyaw = PidController(_PidGains(
            kp=40.0, ki=20.0, kd=2.0, i_limit=300.0, output_limit=1000.0
        ))

    def control_position(self, pos: dict, vel: dict, target: dict, dt: float) -> tuple:
        """Position control mode: cascade PID.

        Args:
            pos: {'x', 'y', 'z', 'rz'} world NED, rz in degrees
            vel: {'x', 'y', 'z', 'rz'} body frame, rz in deg/s
            target: {'x', 'y', 'z', 'rz'} world NED, rz in degrees
            dt: time step in seconds

        Returns:
            (Fx, Fy, Fz, Mz) body-frame forces
        """
        # === XY Cascade ===
        ex_world = target['x'] - pos['x']
        ey_world = target['y'] - pos['y']

        # Polar decomposition
        error_length = math.sqrt(ex_world ** 2 + ey_world ** 2)
        error_angle = math.atan2(ey_world, ex_world)

        # Outer loop: error magnitude -> speed setpoint
        speed_cmd = self.pid_speed_mag.step(error_length, dt)

        # Decompose speed into world-frame velocity
        vx_world_cmd = speed_cmd * math.cos(error_angle)
        vy_world_cmd = speed_cmd * math.sin(error_angle)

        # Rotate to body frame
        yaw_rad = math.radians(pos['rz'])
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        vx_body_cmd = cy * vx_world_cmd + sy * vy_world_cmd
        vy_body_cmd = -sy * vx_world_cmd + cy * vy_world_cmd

        # Inner loop: body-frame velocity PID
        Fx = self.pid_vx.step(vx_body_cmd - vel['x'], dt)
        Fy = self.pid_vy.step(vy_body_cmd - vel['y'], dt)

        # === Z Axis (single position PID) ===
        ez = target['z'] - pos['z']
        Fz = self.pid_z.step(ez, dt)

        # === Yaw Cascade ===
        eyaw = _wrap_angle_deg(target['rz'] - pos['rz'])
        yaw_rate_cmd = self.pid_yaw.step(eyaw, dt)
        yaw_rate_cmd = max(-45.0, min(45.0, yaw_rate_cmd))
        eyaw_rate = yaw_rate_cmd - vel['rz']
        Mz = self.pid_yaw_rate.step(eyaw_rate, dt)

        return Fx, Fy, Fz, Mz

    def control_velocity(self, vel: dict, target_vel: dict, dt: float) -> tuple:
        """Velocity control mode: single-loop body-frame velocity PID.

        Args:
            vel: {'x', 'y', 'z', 'rz'} body frame
            target_vel: {'x', 'y', 'z', 'rz'} body frame
            dt: time step

        Returns:
            (Fx, Fy, Fz, Mz) body-frame forces
        """
        Fx = self.pid_vx.step(target_vel['x'] - vel['x'], dt)
        Fy = self.pid_vy.step(target_vel['y'] - vel['y'], dt)
        Fz = self.pid_vz.step(target_vel['z'] - vel['z'], dt)
        Mz = self.pid_vyaw.step(target_vel['rz'] - vel['rz'], dt)
        return Fx, Fy, Fz, Mz

    def reset(self):
        self.pid_speed_mag.reset()
        self.pid_vx.reset()
        self.pid_vy.reset()
        self.pid_z.reset()
        self.pid_yaw.reset()
        self.pid_yaw_rate.reset()
        self.pid_vz.reset()
        self.pid_vyaw.reset()


class SimBridgeNode(Node):
    """ROS2 node: bridges Stonefish sensors to control layer with cascade PID."""

    def __init__(self):
        super().__init__('sim_bridge')

        # State
        self.pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}  # world NED, rz in degrees
        self.vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}  # body frame
        self.vel_world = {'x': 0.0, 'y': 0.0}  # world frame velocity
        self.target_pos = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.target_vel = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'rz': 0.0}
        self.force_4dof = [0.0, 0.0, 0.0, 0.0]
        self.armed = False
        self.control_mode = 0  # 0=none, 1=pos, 2=vel, 3=force

        # Controllers
        self.pid = CascadePidController()
        self.mixer = ThrustMixer(max_thrust=1000.0)

        # Timing
        self._last_ctrl_time = self.get_clock().now()
        self._cycle_time_ms = 0.0

        # QoS for sensor data (best effort, volatile)
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # === Subscribers: Stonefish sensors ===
        self.create_subscription(Odometry, '/auv/odometry', self._odom_cb, sensor_qos)
        self.create_subscription(Imu, '/auv/imu', self._imu_cb, sensor_qos)
        if DVL is not None:
            self.create_subscription(DVL, '/auv/dvl', self._dvl_cb, sensor_qos)
        self.create_subscription(FluidPressure, '/auv/pressure', self._pressure_cb, sensor_qos)

        # === Subscribers: Camera images ===
        self.create_subscription(Image, '/sim/front_cam/left/image_color', self._front_left_cb, sensor_qos)
        self.create_subscription(Image, '/sim/front_cam/right/image_color', self._front_right_cb, sensor_qos)
        self.create_subscription(Image, '/sim/down_cam/left/image_color', self._down_left_cb, sensor_qos)
        self.create_subscription(Image, '/sim/down_cam/right/image_color', self._down_right_cb, sensor_qos)

        # === Subscriber: Motion command from control layer ===
        self.create_subscription(MotionCommand, '/auv/cmd/motion', self._motion_cmd_cb, 10)

        # === Publishers ===
        self.pub_state = self.create_publisher(AuvState, '/auv/state', 10)
        self.pub_thrusters = self.create_publisher(Float64MultiArray, '/auv/thrusters_cmd', 10)
        self.pub_thrust_debug = self.create_publisher(Float32MultiArray, '/auv/thrust_debug', 10)
        self.pub_pid_debug = self.create_publisher(PidState, '/auv/pid_debug', 10)

        # Camera republishers
        self.pub_front_left = self.create_publisher(Image, '/auv/front_cam/left', 10)
        self.pub_front_right = self.create_publisher(Image, '/auv/front_cam/right', 10)
        self.pub_down_left = self.create_publisher(Image, '/auv/down_cam/left', 10)
        self.pub_down_right = self.create_publisher(Image, '/auv/down_cam/right', 10)
        self.pub_front_stitched = self.create_publisher(Image, '/auv/front_cam/stitched', 10)
        self.pub_down_stitched = self.create_publisher(Image, '/auv/down_cam/stitched', 10)

        # Image buffers (raw republish only, no stitching due to cv_bridge/NumPy ABI issue)
        self._front_left_img = None
        self._front_right_img = None
        self._down_left_img = None
        self._down_right_img = None

        # Timers
        self.create_timer(0.05, self._control_step)   # 20Hz control loop
        self.create_timer(0.05, self._publish_state)   # 20Hz state publish

        self.get_logger().info('SimBridge node started')

    # ========================================================================
    # Sensor callbacks
    # ========================================================================

    def _odom_cb(self, msg: Odometry):
        # Position: direct NED passthrough
        self.pos['x'] = msg.pose.pose.position.x
        self.pos['y'] = msg.pose.pose.position.y
        self.pos['z'] = msg.pose.pose.position.z

        # Yaw from quaternion
        q = msg.pose.pose.orientation
        self.pos['rz'] = math.degrees(_quat_to_yaw(q.x, q.y, q.z, q.w))

        # World-frame velocity (for external use)
        self.vel_world['x'] = msg.twist.twist.linear.x
        self.vel_world['y'] = msg.twist.twist.linear.y

        # Rotate world velocity to body frame
        yaw_rad = math.radians(self.pos['rz'])
        cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
        vx_w = msg.twist.twist.linear.x
        vy_w = msg.twist.twist.linear.y
        self.vel['x'] = cy * vx_w + sy * vy_w
        self.vel['y'] = -sy * vx_w + cy * vy_w
        self.vel['z'] = msg.twist.twist.linear.z

    def _imu_cb(self, msg: Imu):
        # Override yaw rate with IMU data (higher fidelity)
        self.vel['rz'] = math.degrees(msg.angular_velocity.z)

    def _dvl_cb(self, msg):
        # Override body velocity with DVL data (higher fidelity)
        if hasattr(msg, 'velocity'):
            self.vel['x'] = msg.velocity.x
            self.vel['y'] = msg.velocity.y
            self.vel['z'] = msg.velocity.z

    def _pressure_cb(self, msg: FluidPressure):
        pass  # Depth already from odometry

    # ========================================================================
    # Image callbacks (republish + stitch)
    # ========================================================================

    def _front_left_cb(self, msg: Image):
        self._front_left_img = msg
        self.pub_front_left.publish(msg)
        # Publish left image as "stitched" fallback (cv_bridge has NumPy ABI issues)
        self.pub_front_stitched.publish(msg)

    def _front_right_cb(self, msg: Image):
        self._front_right_img = msg
        self.pub_front_right.publish(msg)

    def _down_left_cb(self, msg: Image):
        self._down_left_img = msg
        self.pub_down_left.publish(msg)
        self.pub_down_stitched.publish(msg)

    def _down_right_cb(self, msg: Image):
        self._down_right_img = msg
        self.pub_down_right.publish(msg)

    # ========================================================================
    # Motion command callback
    # ========================================================================

    def _motion_cmd_cb(self, msg: MotionCommand):
        mode = msg.mode
        x, y, z, yaw = msg.x, msg.y, msg.z, msg.yaw
        yaw_deg = math.degrees(yaw)

        if mode == MotionCommand.MODE_POS_WORLD:
            # Absolute world position
            self.target_pos = {'x': x, 'y': y, 'z': z, 'rz': yaw_deg}
            self.control_mode = 1

        elif mode == MotionCommand.MODE_POS_BODY:
            # Body-frame position: rotate offset to world and add to current
            yaw_rad = math.radians(self.pos['rz'])
            cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
            self.target_pos['x'] = self.pos['x'] + cy * x - sy * y
            self.target_pos['y'] = self.pos['y'] + sy * x + cy * y
            self.target_pos['z'] = self.pos['z'] + z
            self.target_pos['rz'] = self.pos['rz'] + yaw_deg
            self.control_mode = 1

        elif mode == MotionCommand.MODE_VEL_WORLD:
            # World-frame velocity: rotate to body
            yaw_rad = math.radians(self.pos['rz'])
            cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
            self.target_vel['x'] = cy * x + sy * y
            self.target_vel['y'] = -sy * x + cy * y
            self.target_vel['z'] = z
            self.target_vel['rz'] = yaw_deg
            self.control_mode = 2

        elif mode == MotionCommand.MODE_VEL_BODY:
            # Body-frame velocity: direct
            self.target_vel = {'x': x, 'y': y, 'z': z, 'rz': yaw_deg}
            self.control_mode = 2

        elif mode == MotionCommand.MODE_FORCE_BODY:
            # Direct force
            self.force_4dof = [x, y, z, yaw]
            self.control_mode = 3

        elif mode == MotionCommand.MODE_POS_WORLD_STEP:
            # Incremental world position step
            self.target_pos['x'] += x
            self.target_pos['y'] += y
            self.target_pos['z'] += z
            self.target_pos['rz'] += yaw_deg
            self.control_mode = 1

        elif mode == MotionCommand.MODE_POS_BODY_STEP:
            # Incremental body position step: rotate to world
            yaw_rad = math.radians(self.pos['rz'])
            cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
            self.target_pos['x'] += cy * x - sy * y
            self.target_pos['y'] += sy * x + cy * y
            self.target_pos['z'] += z
            self.target_pos['rz'] += yaw_deg
            self.control_mode = 1

        elif mode == MotionCommand.MODE_VEL_WORLD_STEP:
            # Incremental world velocity step
            self.target_vel['x'] += x
            self.target_vel['y'] += y
            self.target_vel['z'] += z
            self.target_vel['rz'] += yaw_deg
            self.control_mode = 2

        elif mode == MotionCommand.MODE_VEL_BODY_STEP:
            # Incremental body velocity step
            self.target_vel['x'] += x
            self.target_vel['y'] += y
            self.target_vel['z'] += z
            self.target_vel['rz'] += yaw_deg
            self.control_mode = 2

    # ========================================================================
    # Control loop (20Hz)
    # ========================================================================

    def _control_step(self):
        now = self.get_clock().now()
        dt = (now - self._last_ctrl_time).nanoseconds / 1e9
        self._last_ctrl_time = now
        self._cycle_time_ms = dt * 1000.0

        if dt <= 0.0 or dt > 1.0:
            return

        if self.control_mode == 1:
            # Position control: cascade PID
            fx, fy, fz, mz = self.pid.control_position(
                self.pos, self.vel, self.target_pos, dt
            )
        elif self.control_mode == 2:
            # Velocity control
            fx, fy, fz, mz = self.pid.control_velocity(
                self.vel, self.target_vel, dt
            )
        elif self.control_mode == 3:
            # Force control: pass-through
            fx, fy, fz, mz = self.force_4dof
        else:
            # No control mode: stop
            fx, fy, fz, mz = 0.0, 0.0, 0.0, 0.0

        # Thrust mixing
        thrusters = self.mixer.mix(fx, fy, fz, mz)

        # Publish thruster commands
        msg = Float64MultiArray()
        msg.data = [float(t) for t in thrusters]
        self.pub_thrusters.publish(msg)

        # Debug: thrust forces
        debug_msg = Float32MultiArray()
        debug_msg.data = [float(fx), float(fy), float(fz), float(mz)]
        self.pub_thrust_debug.publish(debug_msg)

    # ========================================================================
    # State publishing (20Hz)
    # ========================================================================

    def _publish_state(self):
        msg = AuvState()
        msg.pos_x = self.pos['x']
        msg.pos_y = self.pos['y']
        msg.pos_z = self.pos['z']
        msg.yaw = math.radians(self.pos['rz'])
        msg.vel_x = self.vel['x']
        msg.vel_y = self.vel['y']
        msg.vel_z = self.vel['z']
        msg.yaw_rate = math.radians(self.vel['rz'])
        msg.vel_world_x = self.vel_world['x']
        msg.vel_world_y = self.vel_world['y']
        msg.armed = self.armed
        msg.control_mode = self.control_mode
        msg.battery_voltage = 16.8  # Simulated
        msg.error_flags = 0
        msg.cycle_time_ms = self._cycle_time_ms
        msg.target_x = self.target_pos['x']
        msg.target_y = self.target_pos['y']
        msg.target_z = self.target_pos['z']
        msg.target_yaw = math.radians(self.target_pos['rz'])
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
