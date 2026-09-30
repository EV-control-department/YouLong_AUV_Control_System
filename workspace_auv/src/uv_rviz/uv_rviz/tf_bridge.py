"""Mirror the canonical AUV TF tree to root topics consumed by Foxy RViz."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_msgs.msg import TFMessage

from auv_protocol.topics import RVIZ_TF, RVIZ_TF_STATIC, TF, TF_STATIC


class TfBridge(Node):
    """Forward /auv/tf and /auv/tf_static without changing frame semantics."""

    def __init__(self):
        super().__init__('uv_rviz_tf_bridge')

        dynamic_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._dynamic_publisher = self.create_publisher(
            TFMessage, RVIZ_TF, dynamic_qos)
        self._static_publisher = self.create_publisher(
            TFMessage, RVIZ_TF_STATIC, static_qos)
        self._dynamic_subscription = self.create_subscription(
            TFMessage, TF, self._forward_dynamic, dynamic_qos)
        self._static_subscription = self.create_subscription(
            TFMessage, TF_STATIC, self._forward_static, static_qos)

        self.get_logger().info(
            f'Mirroring canonical FRD/NED transforms {TF} and {TF_STATIC} '
            f'to RViz TF topics {RVIZ_TF} and {RVIZ_TF_STATIC}')

    def _forward_dynamic(self, message: TFMessage) -> None:
        self._dynamic_publisher.publish(message)

    def _forward_static(self, message: TFMessage) -> None:
        self._static_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = TfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
