"""Mapping consumes only compact camera observations, including empty frames."""

import json
from importlib import import_module
from pathlib import Path
from types import SimpleNamespace

import pytest
import numpy as np
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
        return SimpleNamespace(info=lambda *_: None, warning=lambda *_: None,
                               error=lambda *_: None)


@pytest.mark.parametrize('failed_label', [None, 'mapping:travel_to_cell_4',
                                         'mapping:travel_to_cell_0', 'mapping:reverse_cell_5'])
def test_dead_camera_with_healthy_position_attempts_complete_route_and_records_defaults(monkeypatch, failed_label):
    from uv_task import mapping_task as module
    clock = [0.]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock[0],
        sleep=lambda seconds: clock.__setitem__(0, clock[0]+seconds)))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    params = load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params']
    params.update(require_complete_survey=True, enable_traversal=True)
    node = _Node()
    node.stopped = False
    task = MappingTask(node, params)
    monkeypatch.setattr(task, '_position_ready', lambda: True)
    travels = []
    monkeypatch.setattr(task, '_travel_to', lambda xy, label: travels.append(label) or label != failed_label)
    monkeypatch.setattr(task, '_observe_cones', lambda: pytest.fail('无摄像头不等待视觉'))
    assert task.execute()
    assert not node.stopped
    assert task.attempted_visit_order == params['visit_order']
    assert task.attempted_traversal_order == list(reversed(params['visit_order']))
    assert 'mapping:reverse_return_to_april_tag' in travels
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['config_fallback_mode'] and not payload['verified_complete']
    assert payload['submitted_config'] == params
    assert payload['conclusion']['source'] == 'fallback'
    assert len(payload['conclusion']['assignment']) == 4
    if failed_label:
        assert task.travel_failures[0]['label'] == failed_label
        if 'travel_to_cell' in failed_label:
            cell = int(failed_label.rsplit('_', 1)[1])
            assert cell not in task.visit_order


def test_mapping_position_loss_stops_navigation_not_camera_loss(monkeypatch):
    from uv_task import mapping_task as module
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    task = MappingTask(_Node(), load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params'])
    task.node.stopped = False
    with pytest.raises(RuntimeError, match='定位数据失效'):
        task._continue_navigation()
    assert task.node.stopped


def test_mapping_fixed_yaw_travel_uses_world_segments(monkeypatch):
    params = load_task(Path(__file__).parents[1]/'config/tasks/mapping_grid.json')[0]['params']
    task = MappingTask(_Node(), params)
    monkeypatch.setattr(task, '_ready', lambda: True)
    calls = []
    task.node._follow_world_waypoints = lambda *args: calls.append(args) or True
    assert task._travel_to([1.,2.], '格点测试')
    assert calls[0][0] == [[1.,2.,params['survey_z'],params['survey_yaw_deg']]]
    assert calls[0][-2:] == (.4, 'fixed_yaw')


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
    task.observation_start_stamp = 0.0
    task._stable_at = lambda timestamp: True
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
    task.observation_start_stamp = 0.0
    task._stable_at = lambda timestamp: True
    task._observation_cb(_frame(1, processed=False))
    assert task.perception_stats['rejected_frames'] == 1
    assert task.cell_observations == {}


def test_mapping_excludes_travel_delayed_and_unstable_frames():
    from uv_msgs.msg import PoseInfo
    params = load_task(Path(__file__).parents[1] / 'config/tasks/mapping_grid.json')[0]['params']
    params['allow_motion_observations'] = False
    task = MappingTask(_Node(), params)
    task.current_cell = 4
    for index in range(11):
        pose = PoseInfo()
        pose.stamp.sec = 2 + index // 10
        pose.stamp.nanosec = (index % 10) * 100_000_000
        task._pose_cb(pose)
    assert task._stable_at(3.0)


    task.state = 'travel_to_cell'
    task.observation_start_stamp = 0.0
    task._observation_cb(_frame(3))
    assert not task.cell_observations
    assert task.camera_ready  # Readiness must not require accepting moving observations.
    task.state = 'observe_cell'
    task.observation_start_stamp = 2.9
    task._observation_cb(_frame(2))
    assert not task.cell_observations
    task.pose_history[-1].robot_yaw = 20.0
    task._observation_cb(_frame(3))
    assert not task.cell_observations
    task.pose_history[-1].robot_yaw = 0.0
    task._observation_cb(_frame(3))
    assert task.cell_observations[4]['synchronized_frames'] == 1
    task.pose_history[-1].robot_x = 0.1
    assert not task._stable_at(3.0)
    task.pose_history[-1].robot_x = 0.0
    for pose in task.pose_history:
        pose.robot_yaw = 179.9
    task.pose_history[-1].robot_yaw = -179.9
    assert task._stable_at(3.0)


def test_mapping_accepts_low_dynamic_motion_but_rejects_fast_yaw():
    from uv_msgs.msg import PoseInfo
    params = load_task(Path(__file__).parents[1] / 'config/tasks/mapping_grid.json')[0]['params']
    task = MappingTask(_Node(), params)
    task.state = 'travel_to_cell'
    task.observation_start_stamp = 0.0
    for index in range(11):
        pose = PoseInfo()
        pose.stamp.sec = 2 + index // 10
        pose.stamp.nanosec = (index % 10) * 100_000_000
        pose.robot_x = index * 0.01
        pose.robot_yaw = index * 0.5
        task._pose_cb(pose)
    task._observation_cb(_frame(3))
    assert task.perception_stats['processed_frames'] == 1
    task.pose_history[-1].robot_yaw = 30.0
    task._observation_cb(_frame(3))
    assert task.perception_stats['unstable_frames'] == 1


def test_unverified_fallback_keeps_visual_cells_and_marks_guesses():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    task = MappingTask(_Node(), params)
    task._assume_tag('标记未识别')
    task.final_assignment = task._fill_fallback_assignment({4: 1})
    task.publish_map()

    assert task.final_assignment == {4: 1, 0: 0, 8: 0, 5: 1}
    assert task.fallback_cells == {0, 8, 5}
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['fallback_used'] is True
    assert payload['tag']['id'] == 16
    assert payload['tag']['source'] == 'fallback'
    assert payload['cells'][0]['source'] == 'fallback'
    assert payload['cells'][4]['source'] == 'none'


@pytest.mark.parametrize('enabled', [True, False])
def test_fallback_map_retraces_survey_without_vision_positions(monkeypatch, enabled):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    params['enable_traversal'] = enabled
    task = MappingTask(_Node(), params)
    task.perception_stats['processed_frames'] = 1
    task.camera_ready = True
    task.pose_history.append(object())
    monkeypatch.setattr(task, '_position_ready', lambda: True)
    monkeypatch.setattr(task, '_camera_available', lambda: True)
    monkeypatch.setattr('uv_task.mapping_task.rclpy.ok', lambda: True)
    travels = []
    monkeypatch.setattr(task, '_travel_to',
                        lambda position, label: travels.append((list(position), label)) or True)
    monkeypatch.setattr(task, '_read_tag',
                        lambda: task._assume_tag('标记未识别') or True)
    monkeypatch.setattr(task, '_observe_cones', lambda: False)
    monkeypatch.setattr(task, '_select_final_assignment', lambda: {})
    assert task.execute() is True
    assert task.state == 'complete_with_fallback'
    assert len(task.final_assignment) == 4
    reverse = list(reversed(params['visit_order']))
    assert task.traversal_order == (reverse if enabled else [])
    assert len(travels) == (19 if enabled else 10)
    if enabled:
        assert [position for position, _ in travels[10:-1]] == [
            task.grid_centers[cell][:2].tolist() for cell in reverse[1:]]
        assert travels[-1][0] == [params['tag_x'], params['tag_y']]
    assert task.filters == {}  # Neither fused positions nor avoidance circles required.


def test_conclusion_uses_fallback_even_when_visual_assignment_disagrees():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), load_task(path)[0]['params'])
    task.publish_map()
    assert json.loads(task.conclusion_pub.messages[-1].data)['ready'] is False
    task.final_assignment = {0: 0, 1: 0, 3: 1, 5: 1}
    task.tag_id = 18
    task.tag_filter = SimpleNamespace(position=np.array([1., 2., 1.]),
                                     covariance=np.eye(3), observations=4)
    task.publish_map()
    conclusion = json.loads(task.conclusion_pub.messages[-1].data)
    assert conclusion['ready'] is True
    assert conclusion['assignment'] == {'0': 0, '8': 0, '5': 1, '6': 1}
    assert conclusion['tag']['id'] == 16
    assert conclusion['verified'] is False
    assert conclusion['source'] == 'fallback'
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['tag']['id'] == 18
    assert payload['final_assignment'] == {'0': 0, '1': 0, '3': 1, '5': 1}
    assert payload['conclusion'] == conclusion


def test_invalid_conclusion_does_not_publish_ready_assignment():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    params['fallback_round_cells'] = [2, 8]
    task = MappingTask(_Node(), params)
    task.final_assignment = {0: 0, 1: 0, 3: 1, 5: 1}
    task.publish_map()
    conclusion = json.loads(task.conclusion_pub.messages[-1].data)
    assert not conclusion['ready'] and not conclusion['valid']
    assert conclusion['assignment'] == {}


def test_reverse_traversal_continues_after_failed_waypoint(monkeypatch):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), load_task(path)[0]['params'])
    task.visit_order = list(task.params['visit_order'])
    monkeypatch.setattr(task, '_ready', lambda: True)
    monkeypatch.setattr(task, '_position_ready', lambda: True)
    travels = []
    monkeypatch.setattr(task, '_travel_to',
                        lambda position, label: travels.append(label) or False)
    task._traverse_cones()
    assert len(travels) == 9
    assert len(task.travel_failures) == 9
    assert task.attempted_traversal_order == list(reversed(task.visit_order))
    assert task.traversal_order == [task.visit_order[-1]]


def test_reverse_traversal_requires_completed_first_lap():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), load_task(path)[0]['params'])
    task.visit_order = [8, 7]
    with pytest.raises(RuntimeError, match='巡检路线未完整'):
        task._traverse_cones()


def test_traversal_mission_explicitly_enables_second_lap():
    from uv_task.config_loader import load_mission
    root = Path(__file__).parents[1] / 'config/missions'
    assert load_mission(root/'mapping_grid.json')[-1]['params']['enable_traversal'] is True
    assert load_mission(root/'mapping_grid_traversal.json')[-1]['params']['enable_traversal'] is True


def test_new_run_clears_previous_latched_conclusion():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    node = _Node()
    first = MappingTask(node, params)
    first.final_assignment = {2: 0, 6: 0, 4: 1, 8: 1}
    first.publish_map()
    assert json.loads(first.conclusion_pub.messages[-1].data)['ready']
    second = MappingTask(node, params)
    second.publish_map()
    assert second.conclusion_pub is first.conclusion_pub
    assert not json.loads(second.conclusion_pub.messages[-1].data)['ready']


def test_no_camera_does_not_move_without_a_completed_survey(monkeypatch):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), load_task(path)[0]['params'])
    monkeypatch.setattr(task, '_ready', lambda: False)
    monkeypatch.setattr('uv_task.mapping_task.rclpy.ok', lambda: True)
    monkeypatch.setattr(task, '_travel_to', lambda *_: pytest.fail('未完成首圈不能遍历'))
    assert not task.execute()
    assert task.visit_order == task.traversal_order == []
    assert not json.loads(task.conclusion_pub.messages[-1].data)['ready']


def test_disabled_conclusion_keeps_map_without_creating_result_publisher():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    params['enable_conclusion'] = False
    task = MappingTask(_Node(), params)
    task.publish_map()
    assert task.conclusion_pub is None
    payload = json.loads(task.map_pub.messages[-1].data)
    assert payload['conclusion_enabled'] is False
    assert payload['conclusion'] is None
    assert len(payload['cells']) == 9


def test_disabling_conclusion_revokes_old_latched_result_once():
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    params = load_task(path)[0]['params']
    node = _Node()
    first = MappingTask(node, params)
    first.final_assignment = {2: 0, 6: 0, 4: 1, 8: 1}
    first.publish_map()
    count = len(first.conclusion_pub.messages)
    params = dict(params, enable_conclusion=False)
    second = MappingTask(node, params)
    second.publish_map()
    second.publish_map()
    assert len(first.conclusion_pub.messages) == count+1
    revoked = json.loads(first.conclusion_pub.messages[-1].data)
    assert not revoked['enabled'] and not revoked['ready']
    assert revoked['assignment'] == {}
    third = MappingTask(node, dict(params, enable_conclusion=True))
    third.publish_map()
    assert json.loads(first.conclusion_pub.messages[-1].data)['enabled']


def test_full_flow_requires_survey_instead_of_skipping_to_next_task(monkeypatch):
    path = Path(__file__).parents[1] / 'config/tasks/mapping_grid.json'
    task = MappingTask(_Node(), dict(load_task(path)[0]['params'], require_complete_survey=True))
    monkeypatch.setattr(task, '_ready', lambda: False)
    monkeypatch.setattr('uv_task.mapping_task.rclpy.ok', lambda: True)
    assert not task.execute()
    assert not task.node.stopped  # 普通阶段结束不再无条件杀死整条链。
    assert task.state == 'failed'
    assert not json.loads(task.conclusion_pub.messages[-1].data)['ready']


def test_full_flow_mission_orders_stages_under_one_start():
    from uv_task.config_loader import load_mission
    path = Path(__file__).parents[1] / 'config/missions/full_flow_debug.yaml'
    tasks = load_mission(path)
    assert [task['name'] for task in tasks] == [
        'start', 'mapping_grid', 'turntable', 'grab_sea_cucumber', 'return_origin']
    assert tasks[1]['params']['enable_traversal']
    assert tasks[1]['params']['enable_conclusion']
    assert tasks[1]['params']['require_complete_survey']
    assert tasks[2]['params']['mode'] == 'yolo_bump'
    assert 'disk_pose_odom' in tasks[2]['params']
    assert tasks[-1]['params']['timeout'] == 150.
