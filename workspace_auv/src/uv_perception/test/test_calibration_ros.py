"""Late subscribers receive the exact calibration attached to detections."""
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import DetectionArray, PerceptionCameraInfo
from auv_protocol.topics import PERCEPTION_CAMERA_CALIBRATION, PERCEPTION_DETECTIONS
from uv_camera.image_geometry import EyeUndistorter, CalibrationCache


def test_late_joiner_matches_calibration_to_corrected_detection(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','185')
    context=Context();context.init(args=[])
    publisher_node=subscriber_node=executor=None
    try:
        publisher_node=Node('calibration_publisher_test',context=context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(publisher_node)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher=publisher_node.create_publisher(PerceptionCameraInfo,PERCEPTION_CAMERA_CALIBRATION('down_right'),qos)
        detection_publisher=publisher_node.create_publisher(DetectionArray,PERCEPTION_DETECTIONS,10)
        source=CameraInfo(width=80,height=60,k=[100.,1.2,40.,0.,98.,30.,0.,0.,1.],d=[-.3,.1,.002,0.,0.])
        meta=EyeUndistorter(source,8).message('down_right')
        publisher.publish(meta)
        # The publisher emits metadata only once, before the subscriber exists.
        subscriber_node=Node('late_calibration_consumer_test',context=context)
        executor.add_node(subscriber_node)
        cache=CalibrationCache();received=[]
        subscriber_node.create_subscription(PerceptionCameraInfo,PERCEPTION_CAMERA_CALIBRATION('down_right'),cache.add,qos)
        subscriber_node.create_subscription(DetectionArray,PERCEPTION_DETECTIONS,received.append,10)
        message=DetectionArray(camera_name='down_right',image_space=1,
            calibration_id=meta.calibration_id,camera_info_version=8,capture_id=42,stereo_pair_id=43)
        deadline=time.monotonic()+5.
        while time.monotonic()<deadline:
            detection_publisher.publish(message);executor.spin_once(timeout_sec=.02)
            if received and cache.resolve(received[-1]) is not None:break
        assert received and cache.resolve(received[-1]) is not None
        assert received[-1].capture_id==42 and received[-1].stereo_pair_id==43
        assert not any(cache.resolve(received[-1]).d)
        assert list(cache.resolve(received[-1]).k)==list(source.k)
    finally:
        if executor is not None:executor.shutdown()
        if subscriber_node is not None:subscriber_node.destroy_node()
        if publisher_node is not None:publisher_node.destroy_node()
        context.shutdown()
