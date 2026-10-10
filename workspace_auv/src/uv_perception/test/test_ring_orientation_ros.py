"""Isolated ROS transport of the production detector's new optional fields."""
from types import SimpleNamespace as NS
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from uv_msgs.msg import DetectionArray, LineState
from uv_perception.object_detector import ObjectDetector
from auv_protocol.topics import PERCEPTION_DETECTIONS
from test_ring_orientation import line_image, angular_error


def test_orientation_fields_cross_real_ros_transport(monkeypatch):
    # Separate from any vehicle's default domain; synthetic image only.
    context = Context()
    monkeypatch.setenv('ROS_DOMAIN_ID', '185')
    context.init(args=[])
    node = None
    executor = None
    try:
        node = Node('ring_orientation_transport_test', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        received = []
        node.create_subscription(DetectionArray, PERCEPTION_DETECTIONS,
                                 received.append, 10)
        detector = ObjectDetector.__new__(ObjectDetector)
        detector.node = node
        detector.publisher = node.create_publisher(DetectionArray, PERCEPTION_DETECTIONS, 10)
        detector._line_publishers = {'down_right': NS(publish=lambda _: None)}
        detector._line_state = lambda *_: LineState()
        detector._gate_feature_mode = 'bbox'
        detector._gate_front_class_id = None
        detector._red_ring_class_id = 37
        detector._ring_orientation_enabled = True
        detector._ring_roi_scale = 1.2
        detector._ring_min_pixels = 24
        detector._ring_min_quality = .8
        image, bbox = line_image(120)
        header = NS(timestamp_ns=node.get_clock().now().nanoseconds,
                    capture_id=104, stereo_pair_id=53)
        until = time.monotonic()+5.
        while not received and time.monotonic() < until:
            detector._publish('down', 'right', NS(header=header), image,
                              [(37, .9, bbox)], [None])
            executor.spin_once(timeout_sec=.05)
        assert received, 'No message received in isolated ROS domain'
        message = received[-1]
        assert message.camera_name == 'down_right'
        assert message.capture_id == 104 and message.stereo_pair_id == 53
        stamp_ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        assert stamp_ns == header.timestamp_ns
        result = message.detections[0]
        assert result.orientation_valid
        assert angular_error(result.orientation_axis_deg, 120) <= 3
        assert .8 <= result.orientation_quality <= 1
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.shutdown()
