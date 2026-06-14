"""Vision node: camera image processing and YOLO detection.

Subscribes to camera images, runs YOLO detection, publishes detection results.

Parameters:
    model_path (str): Path to YOLO model .pt file. If empty, searches default paths.
"""

import os
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from uv_msgs.msg import Detection, DetectionArray

# Lazy import cv_bridge to avoid NumPy compatibility issues at module load
CvBridge = None


def _import_cv_bridge():
    global CvBridge
    if CvBridge is None:
        try:
            from cv_bridge import CvBridge as _CB
            CvBridge = _CB
        except Exception as e:
            raise ImportError(f'cv_bridge not available: {e}')


class VisionNode(Node):
    """Vision node: YOLO detection on camera images."""

    def __init__(self):
        super().__init__('vision')

        self.bridge = None
        self._model = None
        self._confidence = 0.5
        self._model_loaded = False
        self._cv_bridge_ok = False

        # Try to import cv_bridge
        try:
            _import_cv_bridge()
            self.bridge = CvBridge()
            self._cv_bridge_ok = True
        except Exception as e:
            self.get_logger().warn(f'cv_bridge not available: {e}')

        # Load YOLO model
        self._load_model()

        # Subscribers
        self.create_subscription(Image, '/auv/front_cam/stitched', self._front_img_cb, 10)
        self.create_subscription(Image, '/auv/down_cam/stitched', self._down_img_cb, 10)

        # Publishers
        self.pub_front_det = self.create_publisher(DetectionArray, '/perception/detection/front', 10)
        self.pub_down_det = self.create_publisher(DetectionArray, '/perception/detection/down', 10)
        self.pub_front_img = self.create_publisher(Image, '/perception/image/front', 10)
        self.pub_down_img = self.create_publisher(Image, '/perception/image/down', 10)

        self.get_logger().info('Vision node started')

    def _load_model(self):
        """Load YOLO model from parameter path or default locations."""
        try:
            from ultralytics import YOLO

            self.declare_parameter('model_path', '')
            model_path = self.get_parameter('model_path').get_parameter_value().string_value

            if not model_path:
                for candidate in [
                    os.path.expanduser('~/YouLong_AUV_Control_System/workspace_auv/src/datas/SAUVC_sim52.pt'),
                ]:
                    if os.path.exists(candidate):
                        model_path = candidate
                        break

            if model_path and os.path.exists(model_path):
                self._model = YOLO(model_path)
                self._model_loaded = True
                self.get_logger().info(f'YOLO model loaded: {model_path}')
            else:
                self.get_logger().warn(f'YOLO model not found, detection disabled')
        except ImportError:
            self.get_logger().warn('ultralytics not installed, detection disabled')

    def _front_img_cb(self, msg: Image):
        """Process front camera image."""
        self._process_image(msg, 'front')

    def _down_img_cb(self, msg: Image):
        """Process down camera image."""
        self._process_image(msg, 'down')

    def _process_image(self, msg: Image, camera: str):
        """Run YOLO detection on image and publish results."""
        if not self._model_loaded or not self._cv_bridge_ok:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        # Run detection
        try:
            results = self._model(cv_img, conf=self._confidence, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        # Build detection array
        det_array = DetectionArray()
        det_array.header = msg.header
        det_array.camera_name = camera

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                det = Detection()
                det.class_id = int(box.cls[0])
                det.confidence = float(box.conf[0])
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                det.bbox_x1 = x1
                det.bbox_y1 = y1
                det.bbox_x2 = x2
                det.bbox_y2 = y2
                det.pixel_x = (x1 + x2) / 2.0
                det.pixel_y = (y1 + y2) / 2.0
                det_array.detections.append(det)

        # Publish
        if camera == 'front':
            self.pub_front_det.publish(det_array)
        else:
            self.pub_down_det.publish(det_array)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
