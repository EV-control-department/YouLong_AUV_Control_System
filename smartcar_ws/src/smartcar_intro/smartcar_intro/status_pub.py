"""Publish a small status message for the ROS 2 communication exercise."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def format_status(count: int) -> str:
    """Return the stable message format used by the exercise."""

    return f"Group online, count={count}"


class StatusPublisher(Node):
    """Publish one status message per second on ``/student_status``."""

    def __init__(self) -> None:
        super().__init__("status_publisher")
        self.publisher = self.create_publisher(String, "/student_status", 10)
        self.count = 0
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self) -> None:
        msg = String()
        msg.data = format_status(self.count)
        self.publisher.publish(msg)
        self.get_logger().info(f"Publish: {msg.data}")
        self.count += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
