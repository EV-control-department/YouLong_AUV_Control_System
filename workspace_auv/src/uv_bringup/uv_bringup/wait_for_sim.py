"""Wait for measured simulator/perception readiness before releasing tasks."""

import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
from uv_msgs.action import BasicMotion
from uv_msgs.msg import DetectionArray, TargetPositionArray


class Readiness:
    """Require multiple real samples; stale startup samples cannot open a gate."""

    def __init__(self, required, max_age=2.0):
        self.required = set(required)
        self.max_age = max_age
        self.samples = {}

    def observe(self, key, now):
        count, last = self.samples.get(key, (0, now))
        self.samples[key] = (count + 1 if now - last <= self.max_age else 1, now)

    def missing(self, now):
        return sorted(key for key in self.required
                      if self.samples.get(key, (0, 0))[0] < 2
                      or now - self.samples[key][1] > self.max_age)


class SimReadyNode(Node):
    def __init__(self, phase, require_ai):
        super().__init__('wait_for_sim_' + phase)
        required = {'odometry', 'control_pose'}
        self.require_control = phase in ('control', 'sensors', 'perception')
        if require_ai and phase == 'sensors':
            required.update('calibration/' + c for c in self.cameras())
        elif require_ai and phase == 'perception':
            required.update('detections/' + c for c in self.cameras())
            required.add('target_positions')
        self.readiness = Readiness(required)
        self.action = ActionClient(self, BasicMotion, 'basic_motion')
        self.create_subscription(Odometry, '/auv/odometry', self._odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, '/zit6/state/pos', self._pose,
                                 qos_profile_sensor_data)
        if require_ai:
            for camera in self.cameras():
                if phase == 'sensors':
                    group, side = camera.split('_')
                    self.create_subscription(
                        CameraInfo, f'/sim/{group}_cam/{side}/camera_info',
                        lambda msg, c=camera: self._calibration(c, msg),
                        qos_profile_sensor_data)
                else:
                    self.create_subscription(
                        DetectionArray, f'/perception/detection/{camera}',
                        lambda msg, c=camera: self.readiness.observe(
                            'detections/' + c, time.monotonic()),
                        qos_profile_sensor_data)
            if phase == 'perception':
                self.create_subscription(
                    TargetPositionArray, '/perception/target_positions',
                    lambda msg: self.readiness.observe('target_positions', time.monotonic()),
                    qos_profile_sensor_data)

    @staticmethod
    def cameras():
        return ('front_left', 'front_right', 'down_left', 'down_right')

    def _odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if all(math.isfinite(v) for v in values) and sum(v*v for v in values[3:]) > 0.5:
            self.readiness.observe('odometry', time.monotonic())

    def _pose(self, msg):
        if len(msg.data) >= 4 and all(math.isfinite(v) for v in msg.data):
            self.readiness.observe('control_pose', time.monotonic())

    def _calibration(self, camera, msg):
        if msg.width > 0 and msg.height > 0 and msg.k[0] > 0 and msg.k[4] > 0:
            self.readiness.observe('calibration/' + camera, time.monotonic())

    def missing(self):
        missing = self.readiness.missing(time.monotonic())
        if self.require_control and not self.action.server_is_ready():
            missing.append('basic_motion action server')
        return missing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--phase', choices=('backend', 'control', 'sensors', 'perception'),
        required=True,
    )
    parser.add_argument('--require-ai', default='true')
    parser.add_argument('--timeout', type=float, default=120.0)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = SimReadyNode(args.phase, args.require_ai.lower() in ('true', '1', 'yes', 'on'))
    start = time.monotonic()
    next_report = start
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            missing = node.missing()
            if not missing:
                node.get_logger().info(
                    f'{args.phase} READY after {time.monotonic() - start:.1f}s')
                return 0
            now = time.monotonic()
            if now - start >= args.timeout:
                node.get_logger().error(
                    f'{args.phase} readiness timed out; missing={missing}; task not started')
                return 1
            if now >= next_report:
                node.get_logger().info(f'{args.phase} waiting for {missing}')
                next_report = now + 5.0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 2
