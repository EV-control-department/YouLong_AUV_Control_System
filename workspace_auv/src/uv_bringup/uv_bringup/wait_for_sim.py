"""Wait for measured simulator/perception readiness before releasing tasks."""

import argparse
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from uv_msgs.action import BasicMotion
from uv_msgs.msg import DetectionArray, PoseInfo, TargetPositionArray
from auv_protocol.topics import (
    BASIC_MOTION, DETECTIONS, TARGETS, STATE_ODOM,
    DOWN_LEFT_INFO, DOWN_RIGHT_INFO, FRONT_LEFT_INFO, FRONT_RIGHT_INFO,
)


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
        self.action = ActionClient(self, BasicMotion, BASIC_MOTION)
        self.create_subscription(PoseInfo, STATE_ODOM, self._odom,
                                 qos_profile_sensor_data)
        if require_ai:
            for camera in self.cameras():
                if phase == 'sensors':
                    info_topics = {
                        'front_left': FRONT_LEFT_INFO,
                        'front_right': FRONT_RIGHT_INFO,
                        'down_left': DOWN_LEFT_INFO,
                        'down_right': DOWN_RIGHT_INFO,
                    }
                    self.create_subscription(
                        CameraInfo, info_topics[camera],
                        lambda msg, c=camera: self._calibration(c, msg),
                        qos_profile_sensor_data)
                else:
                    self.create_subscription(
                        DetectionArray, DETECTIONS(camera),
                        lambda msg, c=camera: self.readiness.observe(
                            'detections/' + c, time.monotonic()),
                        qos_profile_sensor_data)
            if phase == 'perception':
                self.create_subscription(
                    TargetPositionArray, TARGETS,
                    lambda msg: self.readiness.observe('target_positions', time.monotonic()),
                    qos_profile_sensor_data)

    @staticmethod
    def cameras():
        return ('front_left', 'front_right', 'down_left', 'down_right')

    def _odom(self, msg):
        values = (msg.robot_x, msg.robot_y, msg.robot_z, msg.robot_yaw)
        if all(math.isfinite(v) for v in values):
            self.readiness.observe('odometry', time.monotonic())
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
