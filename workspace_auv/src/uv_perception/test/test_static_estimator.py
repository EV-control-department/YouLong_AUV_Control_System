import time
from types import SimpleNamespace

import numpy as np
from builtin_interfaces.msg import Time
from uv_msgs.msg import ObjectMeasurement, ObjectMeasurementArray, ObjectTrack

from uv_perception.object_estimator_static import ObjectEstimator


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeClock:
    def now(self):
        return SimpleNamespace(to_msg=lambda: Time(sec=100, nanosec=0))


class FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, message):
        self.infos.append(message)

    def warning(self, message):
        self.warnings.append(message)


class FakeNode:
    def __init__(self):
        self.publisher = Publisher()
        self.logger = FakeLogger()

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
        return self.logger


def _ray_measurement(observation_id, source_id, class_id, physical, source,
                     origin, target, multi=False, stamp=None, confidence=0.95):
    item = ObjectMeasurement()
    item.observation_id = observation_id
    item.source_detection_ids = [source_id]
    item.class_id = class_id
    item.class_name = physical
    item.physical_class_name = physical
    item.multi_instance = multi
    item.source_camera = source
    item.confidence = confidence
    item.has_ray = True
    item.has_position = False
    direction = np.asarray(target, dtype=float) - np.asarray(origin, dtype=float)
    direction /= np.linalg.norm(direction)
    item.ray_origin_x, item.ray_origin_y, item.ray_origin_z = origin
    item.ray_direction_x, item.ray_direction_y, item.ray_direction_z = direction
    item.ray_sigma_rad = 0.0005
    item.observation_stamp = stamp or Time(sec=observation_id)
    return item


def _array(*measurements):
    message = ObjectMeasurementArray()
    message.header.frame_id = 'odom'
    message.measurements = list(measurements)
    return message


def _add_target_rays(estimator, target, origins, physical='collection_frame',
                     source='front_left', class_id=1, multi=False, offset=0):
    for index, origin in enumerate(origins):
        estimator._measurements(_array(_ray_measurement(
            offset + index + 1, offset + 1000 + index, class_id, physical,
            source, origin, target, multi=multi)))


def _publish(estimator, node):
    estimator._publish()
    return node.publisher.messages[-1]


def test_synthetic_multiview_rays_recover_position_and_stable_track():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    target = np.array([1.2, -0.7, 3.4])
    origins = []
    for index in range(16):
        angle = 2.0 * np.pi * index / 16.0
        origins.append((target[0] + 4.0 * np.cos(angle),
                        target[1] + 4.0 * np.sin(angle),
                        0.5 + 1.5 * (index % 3) / 2.0))

    _add_target_rays(estimator, target, origins)
    output = _publish(estimator, node)

    assert len(output.tracks) == 1
    track = output.tracks[0]
    estimate = np.array([track.world_x, track.world_y, track.world_z])
    assert np.linalg.norm(estimate - target) < 0.03
    assert track.estimate_source == 'front'
    assert track.measurement_count >= 2
    assert track.status == ObjectTrack.STATUS_STABLE
    assert sum(track.position_covariance[index] for index in (0, 4, 8)) <= 0.04


def test_parallel_and_low_parallax_rays_do_not_create_a_position():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    target = np.array([0.0, 0.0, 10.0])
    origins = [(0.0, 0.0, 0.0), (0.01, 0.0, 0.0),
               (0.0, 0.01, 0.0), (0.01, 0.01, 0.0)]

    _add_target_rays(estimator, target, origins)
    output = _publish(estimator, node)

    assert output.tracks == []
    assert len(estimator._pools[('front', 'collection_frame')]) == len(origins)


def test_reversed_rays_that_intersect_only_behind_the_cameras_are_rejected():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    target = np.array([1.0, 0.0, 4.0])
    origins = [(-1.0, -2.0, 0.0), (3.0, -2.0, 0.0),
               (-1.0, 2.0, 0.0), (3.0, 2.0, 1.0)]
    measurements = []
    for index, origin in enumerate(origins):
        item = _ray_measurement(index + 1, 3000 + index, 1,
                                'collection_frame', 'front_left',
                                origin, target)
        direction = -np.asarray((item.ray_direction_x, item.ray_direction_y,
                                 item.ray_direction_z))
        (item.ray_direction_x, item.ray_direction_y,
         item.ray_direction_z) = direction
        measurements.append(item)
    estimator._measurements(_array(*measurements))

    output = _publish(estimator, node)

    assert output.tracks == []
    assert estimator._pools[('front', 'collection_frame')]


def test_front_and_down_views_keep_separate_tracks_for_same_class():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    target = np.array([1.0, 2.0, 3.0])
    origins = [(-1.0, 0.0, 0.0), (3.0, 0.0, 0.0),
               (-1.0, 4.0, 0.0), (3.0, 4.0, 1.0)]
    _add_target_rays(estimator, target, origins, source='front_left', offset=0)
    _add_target_rays(estimator, target, origins, source='down_left', offset=20)

    output = _publish(estimator, node)

    assert len(output.tracks) == 2
    assert {track.estimate_source for track in output.tracks} == {'front', 'down'}
    assert all(track.physical_class_name == 'collection_frame'
               for track in output.tracks)
    assert all(np.linalg.norm(np.array([track.world_x, track.world_y, track.world_z]) -
                              target) < 0.03 for track in output.tracks)


def test_multinstance_gate_separates_two_objects_and_leaves_outlier_as_clutter():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    target_a = np.array([1.0, 0.0, 4.0])
    target_b = np.array([4.0, 1.0, 5.0])
    origins = [(-1.0, -2.0, 0.0), (2.0, -2.0, 0.0),
               (-1.0, 2.0, 0.0), (2.0, 2.0, 1.0),
               (0.0, 0.0, 2.0), (3.0, -1.0, 1.0)]
    _add_target_rays(estimator, target_a, origins, physical='gate', class_id=2,
                     multi=True, offset=0)
    _add_target_rays(estimator, target_b, origins, physical='gate', class_id=2,
                     multi=True, offset=20)
    outlier = _ray_measurement(50, 2050, 2, 'gate', 'front_left',
                               (0.0, 0.0, 0.0), (0.0, 8.0, 1.0), multi=True)
    estimator._measurements(_array(outlier))

    output = _publish(estimator, node)
    positions = [np.array([track.world_x, track.world_y, track.world_z])
                 for track in output.tracks]

    assert len(positions) == 2
    assert min(np.linalg.norm(point - target_a) for point in positions) < 0.05
    assert min(np.linalg.norm(point - target_b) for point in positions) < 0.05


def test_single_instance_keeps_the_strongest_cluster_only():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    target_strong = np.array([1.0, 1.0, 3.0])
    target_weak = np.array([4.0, 1.0, 3.0])
    origins = [(-1.0, 0.0, 0.0), (3.0, 0.0, 0.0),
               (-1.0, 3.0, 0.0), (3.0, 3.0, 1.0), (0.0, 1.0, 1.5)]
    _add_target_rays(estimator, target_strong, origins, offset=0)
    _add_target_rays(estimator, target_weak, origins[:3], offset=20)

    output = _publish(estimator, node)

    assert len(output.tracks) == 1
    estimate = np.array([output.tracks[0].world_x, output.tracks[0].world_y,
                         output.tracks[0].world_z])
    assert np.linalg.norm(estimate - target_strong) < 0.05


def test_single_instance_revises_existing_track_when_stronger_distant_cluster_arrives():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    false_target = np.array([1.0, 0.0, 3.0])
    true_target = np.array([5.0, 0.0, 3.0])
    false_origins = [(-1.0, -1.0, 0.0), (3.0, -1.0, 0.0),
                     (-1.0, 2.0, 0.0), (3.0, 2.0, 1.0)]
    true_origins = []
    for index in range(16):
        angle = 2.0 * np.pi * index / 16.0
        true_origins.append((true_target[0] + 3.0 * np.cos(angle),
                             true_target[1] + 3.0 * np.sin(angle),
                             0.5 + 1.5 * (index % 3) / 2.0))

    _add_target_rays(estimator, false_target, false_origins, offset=0)
    initial = _publish(estimator, node)
    assert len(initial.tracks) == 1
    old_track = initial.tracks[0]
    assert np.linalg.norm(np.array([old_track.world_x, old_track.world_y,
                                    old_track.world_z]) - false_target) < 0.05

    _add_target_rays(estimator, true_target, true_origins, offset=100)
    updated = _publish(estimator, node)

    assert len(updated.tracks) == 1
    track = updated.tracks[0]
    assert track.track_id == old_track.track_id
    assert np.linalg.norm(np.array([track.world_x, track.world_y,
                                    track.world_z]) - true_target) < 0.05
    assert track.measurement_count >= len(true_origins)


def test_full_gate_track_budget_replaces_unmatched_old_track_with_stronger_cluster():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    old_targets = [np.array([float(x), 0.0, 4.0]) for x in (0, 4, 8, 12)]
    relative_origins = [(-1.0, -2.0, 0.0), (1.0, -2.0, 0.0),
                        (-1.0, 2.0, 0.0), (1.0, 2.0, 1.0)]
    for target_index, target in enumerate(old_targets):
        origins = [(target[0] + x, target[1] + y, z)
                   for x, y, z in relative_origins]
        _add_target_rays(estimator, target, origins, physical='gate',
                         class_id=2, multi=True, offset=100 * target_index)
    initial = _publish(estimator, node)
    assert len(initial.tracks) == 4
    old_ids = {track.track_id for track in initial.tracks}
    for target in old_targets:
        assert any(np.linalg.norm(np.array([track.world_x, track.world_y,
                                            track.world_z]) - target) < 0.05
                   for track in initial.tracks)

    new_target = np.array([20.0, 0.0, 4.0])
    new_origins = []
    for index in range(16):
        angle = 2.0 * np.pi * index / 16.0
        new_origins.append((new_target[0] + 3.0 * np.cos(angle),
                            new_target[1] + 3.0 * np.sin(angle),
                            0.5 + 1.5 * (index % 3) / 2.0))
    _add_target_rays(estimator, new_target, new_origins, physical='gate',
                     class_id=2, multi=True, offset=400)
    updated = _publish(estimator, node)

    assert len(updated.tracks) == 4
    new_ids = {track.track_id for track in updated.tracks}
    assert len(old_ids & new_ids) == 3
    assert len(new_ids - old_ids) == 1
    new_track = next(track for track in updated.tracks
                     if track.track_id in new_ids - old_ids)
    assert np.linalg.norm(np.array([new_track.world_x, new_track.world_y,
                                    new_track.world_z]) - new_target) < 0.05
    assert new_track.measurement_count >= len(new_origins)


def test_default_ray_pool_retains_more_than_previous_capacity():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    assert estimator.pool_capacity == 0

    target = np.array([0.0, 0.0, 4.0])
    measurements = [
        _ray_measurement(index + 1, index + 1000, 1,
                         'collection_frame', 'front_left',
                         (-2.0, 0.0, 0.0), target)
        for index in range(301)
    ]
    estimator._measurements(_array(*measurements))

    assert len(estimator._pools[('front', 'collection_frame')]) == 301


def test_ray_only_pool_is_bounded_and_duplicate_detection_is_ignored():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    target = np.array([0.0, 0.0, 4.0])
    origins = [(-2.0, 0.0, 0.0), (2.0, 0.0, 0.0),
               (0.0, 2.0, 0.0), (0.0, -2.0, 0.0)]
    _add_target_rays(estimator, target, origins)
    duplicate = _ray_measurement(1, 1000, 1, 'collection_frame', 'front_left',
                                 origins[0], target)
    estimator._measurements(_array(duplicate))
    estimator._publish()
    assert len(estimator._pools[('front', 'collection_frame')]) == 4
    assert len(node.logger.infos) >= 1
    assert 'clutter=' in node.logger.infos[-1]

    estimator.pool_capacity = 5
    for index in range(5):
        measurement = _ray_measurement(
            100 + index, 2000 + index, 1, 'collection_frame', 'front_left',
            origins[index % len(origins)], target)
        estimator._measurements(_array(measurement))
    assert len(estimator._pools[('front', 'collection_frame')]) == 5


def test_track_quality_and_position_persist_after_pool_stops_receiving_rays():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    target = np.array([1.0, 0.0, 3.0])
    origins = [(-1.0, 0.0, 0.0), (3.0, 0.0, 0.0),
               (-1.0, 3.0, 0.0), (3.0, 3.0, 1.0)]
    _add_target_rays(estimator, target, origins, physical='target_rack',
                     source='down_left', class_id=9)
    initial = _publish(estimator, node)
    assert len(initial.tracks) == 1
    assert initial.tracks[0].physical_class_name == 'target_rack'
    assert initial.tracks[0].estimate_source == 'down'
    initial_track = initial.tracks[0]
    initial_position = (initial_track.world_x, initial_track.world_y,
                        initial_track.world_z)
    initial_stamp = initial_track.last_measurement_stamp
    state = next(iter(estimator._tracks.values()))
    state.last_ns = time.monotonic_ns() - int(3.0e9)

    output = _publish(estimator, node)

    assert len(estimator._tracks) == 1
    assert len(output.tracks) == 1
    track = output.tracks[0]
    assert track.track_id == initial_track.track_id
    assert (track.world_x, track.world_y, track.world_z) == initial_position
    np.testing.assert_array_equal(track.position_covariance,
                                  initial_track.position_covariance)
    assert track.status == initial_track.status
    assert track.last_measurement_stamp == initial_stamp


def test_early_ray_remains_available_for_late_triangulation_after_many_clutter_rays():
    node = FakeNode()
    estimator = ObjectEstimator(node)
    estimator.seed_pair_limit = 4000
    target = np.array([1.0, 0.0, 3.0])
    key = ('front', 'collection_frame')

    estimator._measurements(_array(_ray_measurement(
        1, 1000, 1, 'collection_frame', 'front_left',
        (-1.0, -1.0, 0.0), target, stamp=Time(sec=1))))
    for index in range(80):
        origin = (1000.0, 1000.0 + index, 1000.0)
        endpoint = (1001.0, 1000.0 + index, 1000.0)
        estimator._measurements(_array(_ray_measurement(
            10 + index, 2000 + index, 1, 'collection_frame', 'front_left',
            origin, endpoint)))

    assert _publish(estimator, node).tracks == []

    estimator._measurements(_array(
        _ray_measurement(1000, 3000, 1, 'collection_frame', 'front_left',
                         (3.0, 2.0, 0.0), target, stamp=Time(sec=1000)),
        _ray_measurement(1001, 3001, 1, 'collection_frame', 'front_left',
                         (3.01, 2.0, 0.0), target, stamp=Time(sec=1001))))
    output = _publish(estimator, node)

    assert len(estimator._pools[key]) == 83
    assert len(output.tracks) == 1
    track = output.tracks[0]
    estimate = np.array([track.world_x, track.world_y, track.world_z])
    assert np.linalg.norm(estimate - target) < 0.05
    assert track.measurement_count == 3


def test_failed_origin_reset_keeps_static_estimator_cache():
    from uv_msgs.msg import StateResetResult
    estimator = ObjectEstimator(FakeNode())
    estimator._pools['old'] = ['ray']
    estimator._tracks[9] = object()
    estimator._reset_callback(StateResetResult(success=False, message='armed'))
    assert estimator._pools['old'] == ['ray'] and 9 in estimator._tracks
    estimator._reset_callback(StateResetResult(success=True, origin_generation=2))
    assert not estimator._pools and not estimator._tracks


def test_authoritative_pose_generation_clears_before_result_without_double_clear():
    from uv_msgs.msg import PoseInfo, StateResetResult
    estimator = ObjectEstimator(FakeNode())
    estimator._pools['old'] = ['ray']
    estimator._origin_state_callback(PoseInfo(origin_initialized=True, origin_generation=2))
    assert not estimator._pools
    estimator._pools['new'] = ['new ray']
    estimator._reset_callback(StateResetResult(success=True, origin_generation=2))
    estimator._reset_callback(StateResetResult(success=True, origin_generation=1))
    assert estimator._pools['new'] == ['new ray']
