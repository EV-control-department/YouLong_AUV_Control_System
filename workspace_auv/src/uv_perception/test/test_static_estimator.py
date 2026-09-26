import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from uv_msgs.msg import ObjectMeasurement, ObjectMeasurementArray

from uv_perception.object_estimator_static import ObjectEstimator


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=1, nanosec=0))


class FakeNode:
    def __init__(self):
        self.publisher = Publisher()

    def create_publisher(self, *_args):
        return self.publisher

    def declare_parameter(self, _name, default):
        return SimpleNamespace(value=default)

    def create_subscription(self, *_args):
        return object()

    def create_timer(self, *_args):
        return object()

    def get_clock(self):
        return FakeClock()

    def get_logger(self):
        return SimpleNamespace(warning=lambda _message: None)


def _measurement(observation_id, source_id, class_id, physical, source,
                 point, multi=False):
    item = ObjectMeasurement()
    item.observation_id = observation_id
    item.source_detection_ids = [source_id]
    item.class_id = class_id
    item.class_name = physical
    item.physical_class_name = physical
    item.multi_instance = multi
    item.source_camera = source
    item.confidence = 0.9
    item.has_position = True
    item.world_x, item.world_y, item.world_z = point
    item.position_covariance = [
        0.01, 0.0, 0.0,
        0.0, 0.01, 0.0,
        0.0, 0.0, 0.01,
    ]
    return item


def _array(*measurements):
    message = ObjectMeasurementArray()
    message.header.frame_id = 'odom'
    message.measurements = list(measurements)
    return message


def test_front_and_down_camera_specific_ids_fuse_by_physical_name():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    estimator._measurements(_array(
        _measurement(1, 101, 0, 'collection_frame', 'down_left',
                     (1.0, 2.0, 3.0)),
    ))
    estimator._measurements(_array(
        _measurement(2, 102, 1, 'collection_frame', 'front_left',
                     (1.05, 2.0, 3.0)),
    ))

    assert len(estimator._tracks) == 1
    state = next(iter(estimator._tracks.values()))
    assert state.physical_class_name == 'collection_frame'
    assert state.source_views == {'front', 'down'}
    assert len(state.observation_ids) == 2
    estimator._publish()
    track = node.publisher.messages[-1].tracks[0]
    assert node.publisher.messages[-1].header.frame_id == 'odom'
    assert track.estimate_source == 'front+down'
    assert track.measurement_count == 2


def test_multi_instance_batch_matching_preserves_two_static_landmarks():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    estimator._measurements(_array(
        _measurement(10, 201, 2, 'gate', 'down_left', (0.0, 0.0, 2.0), True),
        _measurement(11, 202, 2, 'gate', 'down_left', (4.0, 0.0, 2.0), True),
    ))
    estimator._measurements(_array(
        _measurement(12, 203, 3, 'gate', 'front_left', (4.1, 0.0, 2.0), True),
        _measurement(13, 204, 3, 'gate', 'front_left', (0.1, 0.0, 2.0), True),
    ))

    assert len(estimator._tracks) == 2
    states = sorted(estimator._tracks.values(), key=lambda state: state.position[0])
    assert states[0].position[0] < 0.2
    assert states[1].position[0] > 3.9
    assert all(len(state.observation_ids) == 2 for state in states)


def test_static_track_is_retained_after_it_becomes_lost():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    estimator._measurements(_array(
        _measurement(20, 301, 9, 'target_rack', 'down_left',
                     (1.0, 0.0, 0.0)),
    ))
    state = next(iter(estimator._tracks.values()))
    state.last_ns = time.monotonic_ns() - int(3.0e9)

    estimator._publish()
    output = node.publisher.messages[-1]
    assert len(estimator._tracks) == 1
    assert len(output.tracks) == 1
    assert output.tracks[0].status == ObjectTrackStatusLost()


def ObjectTrackStatusLost():
    from uv_msgs.msg import ObjectTrack
    return ObjectTrack.STATUS_LOST
