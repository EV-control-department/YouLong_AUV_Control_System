"""Subscribe to the status topic for the ROS 2 communication exercise."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class StatusSubscriber(Node):
    """Print every status message received on ``/student_status``."""

    def __init__(self) -> None:
        super().__init__("status_subscriber")
        self.subscription = self.create_subscription(
            String,
            "/student_status",
            self.listener_callback,
            10,
        )

    def listener_callback(self, msg: String) -> None:
        self.get_logger().info(f"Receive: {msg.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
