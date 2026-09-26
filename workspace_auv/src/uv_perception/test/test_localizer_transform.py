from types import SimpleNamespace
import threading

from builtin_interfaces.msg import Time
from rclpy.clock import ClockType
from sensor_msgs.msg import CameraInfo
from uv_msgs.msg import Detection, DetectionArray

from uv_perception.object_localizer import ObjectLocalizer
from uv_perception.object_localizer_static import PendingPair


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


def test_point_transform_accepts_zero_direction_and_uses_capture_ros_time():
    localizer = ObjectLocalizer.__new__(ObjectLocalizer)
    localizer.world_frame = 'odom'
    localizer._tf_buffer = FakeTransformBuffer()
    stamp = Time(sec=42, nanosec=123)

    point = localizer._to_world((1.0, 2.0, 3.0), 'camera_optical_frame', stamp)

    assert point == (11.0, 1.0, 3.5)
    queried_time = localizer._tf_buffer.args[2]
    assert queried_time.nanoseconds == 42_000_000_123
    assert queried_time.clock_type == ClockType.ROS_TIME
    assert localizer._tf_buffer.kwargs['timeout'].nanoseconds > 0


def test_optical_camera_point_is_rotated_and_translated_into_odom():
    localizer = ObjectLocalizer.__new__(ObjectLocalizer)
    localizer.world_frame = 'odom'
    localizer._tf_buffer = FakeTransformBuffer(
        rotation=SimpleNamespace(x=0.5, y=0.5, z=0.5, w=0.5),
        translation=SimpleNamespace(x=2.1899, y=-0.05, z=0.1780))
    point_camera = (0.0505, 0.2834, 1.1480)

    point_odom = localizer._to_world(
        point_camera, 'front_left_camera_optical_frame', Time(sec=42))

    assert max(abs(a - b) for a, b in zip(point_odom, (3.3379, 0.0005, 0.4614))) < 1e-4

def test_ray_transform_keeps_normalized_world_direction():
    localizer = ObjectLocalizer.__new__(ObjectLocalizer)
    localizer.world_frame = 'odom'
    localizer._tf_buffer = FakeTransformBuffer()
    stamp = Time(sec=1, nanosec=0)

    transformed = localizer._to_world_ray(
        (0.0, 0.0, 0.0), (0.0, 0.0, 2.0), 'camera_optical_frame', stamp)

    assert transformed == ((10.0, -1.0, 0.5), (0.0, 0.0, 1.0))


def test_covariance_is_rotated_into_odom_axes():
    rotation = ((0.0, -1.0, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    covariance = [1.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 9.0]
    assert ObjectLocalizer._rotate_covariance(covariance, rotation) == [
        4.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 9.0]


def test_stereo_factor_transforms_point_and_keeps_both_raw_detection_ids():
    localizer = ObjectLocalizer.__new__(ObjectLocalizer)
    localizer._id_lock = threading.Lock()
    localizer._id_prefix = 17
    localizer._observation_counter = 0
    localizer.baseline_m = 0.10
    localizer.world_frame = 'odom'
    localizer._tf_buffer = FakeTransformBuffer()
    localizer.pose_translation_sigma_m = 0.03
    localizer.pose_rotation_sigma_deg = 1.0
    localizer.extrinsic_translation_sigma_m = 0.005
    localizer.extrinsic_rotation_sigma_deg = 0.5

    info = CameraInfo()
    info.width, info.height = 1280, 960
    info.k = [500.0, 0.0, 640.0, 0.0, 500.0, 480.0, 0.0, 0.0, 1.0]

    left = DetectionArray()
    left.header.frame_id = 'front_left_camera_optical_frame'
    left.header.stamp = Time(sec=42, nanosec=0)
    left.camera_name = 'front_left'
    left.capture_id = 8
    left.stereo_pair_id = 8
    left_detection = Detection()
    left_detection.class_id = 1
    left_detection.confidence = 0.9
    left_detection.pixel_x, left_detection.pixel_y = 600.0, 400.0
    left.detections = [left_detection]

    right = DetectionArray()
    right.header.frame_id = 'front_right_camera_optical_frame'
    right.header.stamp = left.header.stamp
    right.camera_name = 'front_right'
    right.capture_id = 8
    right.stereo_pair_id = 8
    right_detection = Detection()
    right_detection.class_id = 1
    right_detection.confidence = 0.9
    right_detection.pixel_x, right_detection.pixel_y = 580.0, 400.0
    right.detections = [right_detection]

    measurement = localizer._stereo_measurement(
        left, 0, right, 0, info, info, 501, 502)

    assert measurement is not None and measurement.has_position
    assert list(measurement.source_detection_ids) == [501, 502]
    assert abs(measurement.world_x - 9.8) < 1e-6
    assert abs(measurement.world_y + 1.4) < 1e-6
    assert abs(measurement.world_z - 3.0) < 1e-6
    assert measurement.position_covariance[0] > 0.0

def test_publishing_measurement_header_does_not_mutate_camera_header():
    localizer = ObjectLocalizer.__new__(ObjectLocalizer)
    localizer.world_frame = 'odom'
    left = DetectionArray()
    left.header.frame_id = 'front_left_camera_optical_frame'
    left.camera_name = 'front_left'
    localizer._publish_pending(
        PendingPair(first_seen_ns=0, messages={'front_left': left}), {})
    assert left.header.frame_id == 'front_left_camera_optical_frame'


def test_world_frame_as_camera_source_is_rejected():
    localizer = ObjectLocalizer.__new__(ObjectLocalizer)
    localizer.world_frame = 'odom'
    localizer._tf_buffer = FakeTransformBuffer()
    assert localizer._to_world((0.1, 0.2, 1.0), 'odom', Time(sec=42)) is None
