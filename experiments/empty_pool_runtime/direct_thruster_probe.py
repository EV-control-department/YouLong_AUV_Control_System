import argparse
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Probe(Node):
    def __init__(self, index=0, sign=1.0):
        super().__init__('direct_thruster_probe')
        self.command = self.create_publisher(
            Float64MultiArray, '/auv/sim/actuators/thruster_command', 10)
        self.create_subscription(
            Odometry, '/auv/sim/ground_truth/odom', self.odom_cb, 10)
        self.t0 = time.monotonic()
        self.index = index
        self.sign = sign
        self.last = 0.0
        self.odom = None
        self.timer = self.create_timer(0.01, self.tick)

    def odom_cb(self, msg):
        p = msg.pose.pose
        v = msg.twist.twist
        self.odom = (p.position.x, p.position.y, p.position.z,
                     v.linear.x, v.linear.y, v.linear.z,
                     v.angular.x, v.angular.y, v.angular.z)

    def tick(self):
        elapsed = time.monotonic() - self.t0
        phase = int(elapsed // 1.2)
        command = [0.0] * 6
        if phase == 1:
            command[self.index] = 0.5 * self.sign
        msg = Float64MultiArray()
        msg.data = command
        self.command.publish(msg)
        if elapsed - self.last >= 0.1:
            self.last = elapsed
            self.get_logger().info('t=%.2f phase=%d cmd=%s odom=%s' %
                                   (elapsed, phase, command, self.odom))
        if elapsed >= 3.0:
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--index', type=int, default=0)
    parser.add_argument('--sign', type=float, default=1.0)
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args.index, args.sign)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
