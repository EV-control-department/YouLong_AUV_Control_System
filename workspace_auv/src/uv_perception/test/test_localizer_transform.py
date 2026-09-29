from types import SimpleNamespace
import threading

from builtin_interfaces.msg import Time
from rclpy.clock import ClockType
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import Detection, DetectionArray

from uv_perception.object_localizer import ObjectLocalizer
from uv_perception.object_localizer_static import PendingPair
from uv_perception.localization.stereo import camera_ray


class FakeTransformBuffer:
    def __init__(self, rotation=None, translation=None):
        from std_msgs.msg import Header
        self.args = None
        self.kwargs = None
        rotation = rotation or SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
        translation = translation or SimpleNamespace(x=10.0, y=-1.0, z=0.5)
        self.transform = SimpleNamespace(
            header=Header(frame_id='odom'),
            transform=SimpleNamespace(rotation=rotation, translation=translation))

    def lookup_transform(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        return self.transform


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _localizer():
    localizer = ObjectLocalizer.__new__(ObjectLocalizer)
    localizer._id_lock = threading.Lock()
    localizer._id_prefix = 17
    localizer._observation_counter = 0
    localizer.world_frame = 'odom'
    localizer._tf_buffer = FakeTransformBuffer()
    localizer._warned_tf_frames = set()
    localizer.extrinsic_rotation_sigma_deg = 0.5
    localizer.edge_margin_px = 8.0
    localizer.edge_margin_ratio = 0.02
    localizer.publisher = Publisher()
    return localizer


def _camera_info():
    info = CameraInfo()
    info.width, info.height = 1280, 960
    info.k = [500.0, 0.0, 640.0, 0.0, 500.0, 480.0, 0.0, 0.0, 1.0]
    return info


def _detection(class_id=1):
    detection = Detection()
    detection.class_id = class_id
    detection.confidence = 0.9
    detection.pixel_x, detection.pixel_y = 600.0, 400.0
    detection.bbox_x1, detection.bbox_y1 = 580.0, 380.0
    detection.bbox_x2, detection.bbox_y2 = 620.0, 420.0
    return detection


def test_point_transform_accepts_zero_direction_and_uses_capture_ros_time():
    localizer = _localizer()
    stamp = Time(sec=42, nanosec=123)

    point = localizer._to_world((1.0, 2.0, 3.0), 'camera_optical_frame', stamp)

    assert point == (11.0, 1.0, 3.5)
    queried_time = localizer._tf_buffer.args[2]
    assert queried_time.nanoseconds == 42_000_000_123
    assert queried_time.clock_type == ClockType.ROS_TIME
    assert localizer._tf_buffer.kwargs['timeout'].nanoseconds > 0


def test_optical_camera_point_is_rotated_and_translated_into_odom():
    localizer = _localizer()
    localizer._tf_buffer = FakeTransformBuffer(
        rotation=SimpleNamespace(x=0.5, y=0.5, z=0.5, w=0.5),
        translation=SimpleNamespace(x=2.1899, y=-0.05, z=0.1780))
    point_camera = (0.0505, 0.2834, 1.1480)

    point_odom = localizer._to_world(
        point_camera, 'front_left_camera_optical_frame', Time(sec=42))

    assert max(abs(a - b) for a, b in zip(point_odom, (3.3379, 0.0005, 0.4614))) < 1e-4


def test_ray_transform_keeps_normalized_world_direction():
    localizer = _localizer()
    stamp = Time(sec=1, nanosec=0)

    transformed = localizer._to_world_ray(
        (0.0, 0.0, 0.0), (0.0, 0.0, 2.0), 'camera_optical_frame', stamp)

    assert transformed == ((10.0, -1.0, 0.5), (0.0, 0.0, 1.0))


def test_detection_feature_pixel_is_used_for_ray_and_position_is_not_created():
    localizer = _localizer()
    info = _camera_info()
    message = DetectionArray()
    message.header.frame_id = 'front_left_camera_optical_frame'
    message.header.stamp = Time(sec=42)
    message.camera_name = 'front_left'
    detection = _detection()
    detection.feature_type = Detection.FEATURE_GATE_CENTERLINE
    detection.feature_pixel_x, detection.feature_pixel_y = 700.0, 500.0
    message.detections = [detection]

    measurement = localizer._bearing_measurement(message, 0, info, 501)
    expected = camera_ray(info, 700.0, 500.0)

    assert measurement.has_ray and not measurement.has_position
    assert measurement.measurement_form == measurement.FORM_FRONT_BEARING
    assert list(measurement.source_detection_ids) == [501]
    assert measurement.ray_origin_x == 10.0
    assert max(abs(a - b) for a, b in zip(
        (measurement.ray_direction_x, measurement.ray_direction_y,
         measurement.ray_direction_z), expected)) < 1e-6


def test_bbox_center_is_the_fallback_when_optional_feature_is_absent():
    localizer = _localizer()
    info = _camera_info()
    message = DetectionArray()
    message.header.frame_id = 'front_left_camera_optical_frame'
    message.header.stamp = Time(sec=42)
    message.camera_name = 'front_left'
    detection = _detection()
    detection.feature_type = Detection.FEATURE_BBOX_CENTER
    detection.pixel_x, detection.pixel_y = 900.0, 700.0
    message.detections = [detection]

    measurement = localizer._bearing_measurement(message, 0, info, 500)
    expected = camera_ray(info, 600.0, 400.0)

    assert max(abs(a - b) for a, b in zip(
        (measurement.ray_direction_x, measurement.ray_direction_y,
         measurement.ray_direction_z), expected)) < 1e-6


def test_front_stereo_pair_publishes_two_independent_rays():
    localizer = _localizer()
    info = _camera_info()
    left = DetectionArray()
    left.header.frame_id = 'front_left_camera_optical_frame'
    left.header.stamp = Time(sec=42)
    left.camera_name = 'front_left'
    left.detections = [_detection()]
    right = DetectionArray()
    right.header.frame_id = 'front_right_camera_optical_frame'
    right.header.stamp = Time(sec=42)
    right.camera_name = 'front_right'
    right.detections = [_detection()]
    pending = PendingPair(
        first_seen_ns=0,
        messages={'front_left': left, 'front_right': right},
        detection_ids={'front_left': [501], 'front_right': [502]})

    localizer._publish_pending(pending, {'front_left': info, 'front_right': info})

    output = localizer.publisher.messages[-1]
    assert len(output.measurements) == 2
    assert [list(item.source_detection_ids) for item in output.measurements] == [[501], [502]]
    assert all(item.has_ray and not item.has_position for item in output.measurements)
    assert all(item.measurement_form == item.FORM_FRONT_BEARING
               for item in output.measurements)


def test_down_view_uses_its_own_bearing_form_without_ground_intersection():
    localizer = _localizer()
    message = DetectionArray()
    message.header.frame_id = 'down_left_camera_optical_frame'
    message.header.stamp = Time(sec=42)
    message.camera_name = 'down_left'
    message.detections = [_detection(class_id=0)]

    measurement = localizer._bearing_measurement(message, 0, _camera_info(), 601)

    assert measurement.has_ray and not measurement.has_position
    assert measurement.measurement_form == measurement.FORM_DOWN_BEARING


def test_publishing_measurement_header_does_not_mutate_camera_header():
    localizer = _localizer()
    left = DetectionArray()
    left.header.frame_id = 'front_left_camera_optical_frame'
    left.camera_name = 'front_left'
    localizer._publish_pending(
        PendingPair(first_seen_ns=0, messages={'front_left': left}), {})
    assert left.header.frame_id == 'front_left_camera_optical_frame'


def test_world_frame_as_camera_source_is_rejected():
    localizer = _localizer()
    assert localizer._to_world((0.1, 0.2, 1.0), 'odom', Time(sec=42)) is None
