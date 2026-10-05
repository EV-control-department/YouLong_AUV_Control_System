"""Publish the detector class mapping as a latched ROS 2 topic."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from auv_protocol.topics import MODEL_CLASS_MAPPING
from uv_msgs.msg import ModelClassMapping

from .model_classes import CLASS_METADATA, MODEL_MAPPING, MODEL_MAPPING_PATH


MAPPING_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def mapping_message():
    message = ModelClassMapping()
    message.schema_version = int(MODEL_MAPPING.get('schema_version', 1))
    message.model = str(MODEL_MAPPING.get('model', MODEL_MAPPING_PATH.stem + '.pt'))
    for class_id in sorted(CLASS_METADATA):
        entry = CLASS_METADATA[class_id]
        message.ids.append(int(class_id))
        message.names.append(str(entry['name']))
        message.objects.append(str(entry.get('object', entry['name'])))
        message.cameras.append(str(entry.get('camera') or ''))
        message.multi_instance.append(bool(entry.get('multi_instance', False)))
    return message


class ModelClassPublisher(Node):
    def __init__(self):
        super().__init__('model_class_publisher')
        self.publisher = self.create_publisher(ModelClassMapping,
                                                MODEL_CLASS_MAPPING,
                                                MAPPING_QOS)
        self.message = mapping_message()
        self.publisher.publish(self.message)
        self.get_logger().info(
            'published detector mapping {} ({} classes) on {}'.format(
                self.message.model, len(self.message.ids), MODEL_CLASS_MAPPING))


def main(args=None):
    rclpy.init(args=args)
    node = ModelClassPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
