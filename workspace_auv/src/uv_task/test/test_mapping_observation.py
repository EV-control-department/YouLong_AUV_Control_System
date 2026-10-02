"""Mapping consumes only compact camera observations, including empty frames."""

import json
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace

import pytest
from uv_msgs.msg import MappingObservation, MappingObservationArray
from uv_task.config_loader import load_task
from uv_task.mapping_task import MappingTask


def test_mapping_runner_import_does_not_require_unrelated_competition_modules():
    runner = import_module('uv_task.task_runner')
    assert callable(runner.TaskRunnerNode._task_mapping_grid)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Node:
    stopped = False

    def create_publisher(self, *_args):
        return _Publisher()

    def create_subscription(self, *_args):
        return object()

    def create_timer(self, *_args):
        return object()

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))

    def get_logger(self):
        return SimpleNamespace(info=lambda *_: None, warning=lambda *_: None)


def _frame(stamp, *observations, processed=True):
    frame = MappingObservationArray()
    frame.header.stamp.sec = stamp
    frame.processed = processed
    frame.reason = 'ok' if processed else 'camera calibration unavailable'
    frame.observations = list(observations)
    return frame


def test_mapping_associates_and_filters_compact_observations():
    params = {
        'timeout': 60, 'grid_center_x': 2, 'grid_center_y': -4,
        'floor_z': 1.994, 'grid_yaw_deg': 0, 'grid_side_m': 2,
        'tag_id': -1, 'measurement_sigma_m': 0.035,
        'process_noise': 0.0001, 'mahalanobis_gate': 11.345,
        'cell_gate_m': 0.48, 'min_observations': 2,
        'class_vote_ratio': 0.67, 'expected_cones': 4,
    }
    task = MappingTask(_Node(), params)
    task.current_cell = 4
    task.state = 'observe_cell'
    task._observation_cb(_frame(1))
    cone = MappingObservation()
    cone.kind = MappingObservation.CONE
    cone.class_id = 1
    cone.confidence = 0.9
    cone.depth_m = 1.5
    cone.depth_samples = 50
    cone.world_x, cone.world_y, cone.world_z = 2.01, -4.02, 1.8
    task._observation_cb(_frame(2, cone))
    cone.world_x = 2.02
    task._observation_cb(_frame(3, cone))
    assert task.cell_observations[4]['synchronized_frames'] == 3
    assert task.filters[4].accepted == 2
    assert task.class_votes[4] == [0, 2]
    assert task.perception_stats['processed_frames'] == 3


def test_rejected_camera_frame_is_not_counted_as_synchronized():
    params = {
        'timeout': 60, 'grid_center_x': 2, 'grid_center_y': -4,
        'floor_z': 1.994, 'grid_yaw_deg': 0, 'grid_side_m': 2,
        'tag_id': -1, 'measurement_sigma_m': 0.035,
        'process_noise': 0.0001, 'mahalanobis_gate': 11.345,
        'cell_gate_m': 0.48, 'min_observations': 2,
        'class_vote_ratio': 0.67, 'expected_cones': 4,
    }
    task = MappingTask(_Node(), params)
    task.current_cell = 4
    task.state = 'observe_cell'
    task._observation_cb(_frame(1, processed=False))
    assert task.perception_stats['rejected_frames'] == 1
    assert task.cell_observations == {}


def test_unverified_fallback_keeps_visual_cells_and_marks_guesses():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    task = MappingTask(_Node(), params)
    task._assume_tag('标记未识别')
    task.final_assignment = task._fill_fallback_assignment({4: 1})
    task.publish_map()

    assert task.final_assignment == {4: 1, 2: 0, 6: 0, 8: 1}
    assert task.fallback_cells == {2, 6, 8}
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['fallback_used'] is True
    assert payload['tag']['id'] == 16
    assert payload['tag']['source'] == 'fallback'
    assert payload['cells'][2]['source'] == 'fallback'
    assert payload['cells'][4]['source'] == 'none'


def test_fallback_map_does_not_trigger_cone_traversal(monkeypatch):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    task = MappingTask(_Node(), params)
    task.perception_stats['processed_frames'] = 1
    task.pose_history.append(object())
    monkeypatch.setattr('uv_task.mapping_task.rclpy.ok', lambda: True)
    monkeypatch.setattr(task, '_travel_to', lambda *_: True)
    monkeypatch.setattr(task, '_read_tag',
                        lambda: task._assume_tag('标记未识别') or True)
    monkeypatch.setattr(task, '_observe_cones', lambda: False)
    monkeypatch.setattr(task, '_select_final_assignment', lambda: {})
    monkeypatch.setattr(task, '_traverse_cones',
                        lambda: pytest.fail('猜测地图不得触发自动遍历'))

    assert task.execute() is True
    assert task.state == 'complete_with_fallback'
    assert len(task.final_assignment) == 4
    assert task.traversal_order == []
