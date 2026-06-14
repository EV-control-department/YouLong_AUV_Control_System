"""Hardware manager node: placeholder for real MCU communication.

This node will eventually handle:
- Serial communication with STM32 MCU
- micro-ROS service for parameter modification
- PID parameter read/write
- Thrust curve configuration
- Device control (LED, servo, arm)

For now, it's a placeholder that publishes a heartbeat.
"""

import rclpy
from rclpy.node import Node


class HwManagerNode(Node):
    """Hardware manager placeholder node."""

    def __init__(self):
        super().__init__('hw_manager')
        self.get_logger().info('HW Manager node started (placeholder)')


def main(args=None):
    rclpy.init(args=args)
    node = HwManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
