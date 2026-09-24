"""YOLO detector: iceoryx2 stitched frames to one aggregate ROS topic."""

from __future__ import annotations

from pathlib import Path
import os
import threading

from auv_protocol.topics import ICEORYX_CAMERA_DOWN, ICEORYX_CAMERA_FRONT, PERCEPTION_DETECTIONS
from uv_msgs.msg import Detection, DetectionArray

from .detector.yolo_detector import YoloDetector
from .transport.iceoryx2 import Iceoryx2Reader


def _model_default() -> str:
    override = os.environ.get('UV_YOLO_MODEL', '').strip()
    if override:
        return override
    here = Path(__file__).resolve()
    for parent in (here.parent, *here.parents):
        for candidate in (
                parent / 'uv_camera' / 'weights' / 'robotcup20260901.pt',
                parent / 'workspace_auv' / 'src' / 'uv_camera' / 'weights' /
                'robotcup20260901.pt'):
            if candidate.is_file():
                return str(candidate)
    try:
        from ament_index_python.packages import get_package_share_directory
        candidate = (Path(get_package_share_directory('uv_camera')) /
                     'weights' / 'robotcup20260901.pt')
        if candidate.is_file():
            return str(candidate)
    except Exception:
        pass
    return ''


def _stamp(node, timestamp_ns):
    from rclpy.time import Time
    return Time(nanoseconds=int(timestamp_ns)).to_msg()


class ObjectDetector:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(DetectionArray, PERCEPTION_DETECTIONS, 10)
        self.confidence = float(node.declare_parameter('confidence', 0.5).value)
        self.device = str(node.declare_parameter('device', '').value or '')
        configured_model = str(
            node.declare_parameter('model_path', _model_default()).value or '').strip()
        model_path = configured_model or _model_default()
        self._detector = None
        if model_path and Path(model_path).is_file():
            try:
                self._detector = YoloDetector(model_path, self.confidence, self.device)
                node.get_logger().info(f'object_detector loaded YOLO model {model_path}')
            except Exception as error:
                node.get_logger().error(f'cannot load YOLO model: {error}')
        else:
            node.get_logger().warning(
                'object_detector has no model; it will publish empty detection arrays')
        self._stop = threading.Event()
        self._threads = []
        for camera, service in (('front', ICEORYX_CAMERA_FRONT),
                                ('down', ICEORYX_CAMERA_DOWN)):
            thread = threading.Thread(target=self._read_loop,
                                      args=(camera, service),
                                      name=f'detector-{camera}', daemon=True)
            self._threads.append(thread)
            thread.start()

    def _read_loop(self, camera, service):
        reader = None
        try:
            reader = Iceoryx2Reader(service)
            while not self._stop.is_set():
                packet = reader.read()
                if packet is None:
                    return
                image = packet.bgr()
                half_width = image.shape[1] // 2
                for side, offset in (('left', 0), ('right', half_width)):
                    eye = image[:, offset:offset + half_width]
                    results = self._detector.detect(eye) if self._detector else ()
                    self._publish(camera, side, packet, results)
        except Exception as error:
            self.node.get_logger().error(f'{camera} detector loop stopped: {error}')
        finally:
            if reader is not None:
                reader.close()

    def _publish(self, camera, side, packet, results):
        message = DetectionArray()
        message.header.stamp = _stamp(self.node, packet.header.timestamp_ns)
        message.header.frame_id = f'{camera}_{side}_camera_optical_frame'
        message.camera_name = f'{camera}_{side}'
        message.capture_id = packet.header.capture_id
        message.stereo_pair_id = packet.header.stereo_pair_id
        for class_id, confidence, box in results:
            detection = Detection()
            detection.class_id = int(class_id)
            detection.confidence = float(confidence)
            detection.bbox_x1 = float(box[0])
            detection.bbox_y1 = float(box[1])
            detection.bbox_x2 = float(box[2])
            detection.bbox_y2 = float(box[3])
            detection.pixel_x = (detection.bbox_x1 + detection.bbox_x2) * 0.5
            detection.pixel_y = (detection.bbox_y1 + detection.bbox_y2) * 0.5
            detection.feature_pixel_x = detection.pixel_x
            detection.feature_pixel_y = detection.pixel_y
            message.detections.append(detection)
        self.publisher.publish(message)

    def close(self):
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=1.0)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    rclpy.init(args=args)
    node = rclpy.create_node('object_detector')
    detector = ObjectDetector(node)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        detector.close()
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
