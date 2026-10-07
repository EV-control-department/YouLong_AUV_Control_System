"""Wait for a healthy ZIT6 backend before an auto-started simulation mission."""

import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from zit6_interfaces.msg import ZitOdom, ZitStatus
from zit6_interfaces.srv import SetOrigin
from uv_msgs.action import BasicMotion
from auv_protocol.topics import BASIC_MOTION, ZIT6_ODOM, ZIT6_SET_ORIGIN, ZIT6_STATUS


class BackendReadyNode(Node):
    def __init__(self):
        super().__init__('wait_for_sim_backend')
        self.declare_parameter('timeout', 120.0)
        self._odom_samples = 0
        self._last_odom_s = None
        self._status = None
        self._last_status_s = None
        self.action = ActionClient(self, BasicMotion, BASIC_MOTION)
        self.set_origin = self.create_client(SetOrigin, ZIT6_SET_ORIGIN)
        self.create_subscription(ZitOdom, ZIT6_ODOM, self._odom_cb, 10)
        self.create_subscription(ZitStatus, ZIT6_STATUS, self._status_cb, 10)

    def _odom_cb(self, message):
        if message.nav_valid and all(math.isfinite(v) for v in message.pose_odom):
            now_s = time.monotonic()
            self._odom_samples = (self._odom_samples + 1
                                  if self._last_odom_s is not None
                                  and now_s - self._last_odom_s <= 2.0 else 1)
            self._last_odom_s = now_s
        else:
            self._odom_samples = 0

    def _status_cb(self, message):
        self._status = message
        self._last_status_s = time.monotonic()

    def ready(self):
        now_s = time.monotonic()
        return (self._odom_samples >= 2 and self._last_odom_s is not None
                and now_s - self._last_odom_s <= 2.0
                and self._status is not None and self._status.navigation_ready
                and self._last_status_s is not None
                and now_s - self._last_status_s <= 2.0
                and self.set_origin.service_is_ready()
                and self.action.server_is_ready())


def main(args=None):
    rclpy.init(args=args)
    node = BackendReadyNode()
    deadline = time.monotonic() + float(node.get_parameter('timeout').value)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                node.get_logger().info('ZIT6 backend and BasicMotion ready')
                return 0
        node.get_logger().error('ZIT6 backend readiness timed out; mission not started')
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
