"""Compatibility checks run against the real Foxy/Python 3.8 dependencies."""
import ast
import importlib
import inspect
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest
from rclpy.logging import get_logger
from rclpy.node import Node
from uv_camera.camera_config import load_camera_config
from uv_task.config_loader import load_mission_or_task
from uv_task.hit_ball_config import DEFAULTS as HIT_DEFAULTS

ROOT = Path(__file__).parents[1]
MODULES = sorted((ROOT / 'uv_task').glob('*.py'))
STDLIB = {'collections', 'dataclasses', 'functools', 'itertools', 'json',
          'math', 'os', 'pathlib', 'threading', 'time', 'typing', 'asyncio'}
LOG_LEVELS = {'debug', 'info', 'warn', 'warning', 'error', 'fatal', 'critical'}


@pytest.mark.parametrize('path', MODULES, ids=lambda path: path.name)
def test_module_imports(path):
    name = 'uv_task' if path.stem == '__init__' else 'uv_task.' + path.stem
    module = importlib.import_module(name)
    assert Path(module.__file__).resolve() == path.resolve()


@pytest.mark.parametrize('path', MODULES, ids=lambda path: path.name)
def test_no_unsupported_python_or_dynamic_log_calls(path):
    tree = ast.parse(path.read_text(encoding='utf-8'), feature_version=8)
    compile(tree, str(path), 'exec')
    aliases = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.split('.')[0] in STDLIB:
                    aliases[alias.asname or alias.name] = importlib.import_module(alias.name)
        elif isinstance(node, ast.ImportFrom) and node.module in STDLIB:
            module = importlib.import_module(node.module)
            for alias in node.names:
                aliases[alias.asname or alias.name] = getattr(module, alias.name)

    for node in ast.walk(tree):
        if isinstance(node, ast.IfExp):
            assert not any(isinstance(part, ast.Attribute) and part.attr in LOG_LEVELS
                           for part in ast.walk(node)), (
                f'{path.name}:{node.lineno}: select log levels at separate call sites')
        if not isinstance(node, ast.Call):
            continue
        function = node.func
        if isinstance(function, ast.Attribute):
            assert function.attr not in {'removeprefix', 'removesuffix'}, (
                f'{path.name}:{node.lineno}: this string API requires Python 3.9')
            attributes = []
            value = function
            while isinstance(value, ast.Attribute):
                attributes.append(value.attr)
                value = value.value
            if isinstance(value, ast.Name) and value.id in aliases:
                target = aliases[value.id]
                for attribute in reversed(attributes):
                    assert hasattr(target, attribute), (
                        f'{path.name}:{node.lineno}: {value.id}.{attribute} is unavailable')
                    target = getattr(target, attribute)
        if isinstance(function, ast.Name) and function.id == 'zip':
            assert all(keyword.arg != 'strict' for keyword in node.keywords)
        if isinstance(function, ast.Name) and function.id == 'dataclass':
            assert all(keyword.arg not in {'slots', 'kw_only'} for keyword in node.keywords)


def test_all_registered_task_handlers_exist():
    from uv_task.task_runner import TaskRunnerNode

    tree = ast.parse((ROOT / 'uv_task/task_runner.py').read_text(encoding='utf-8'))
    assignment = next(node for node in ast.walk(tree)
                      if isinstance(node, ast.Assign) and any(
                          isinstance(target, ast.Attribute) and target.attr == 'task_map'
                          for target in node.targets))
    assert len({key.value for key in assignment.value.keys}) == len(assignment.value.keys)
    for handler in assignment.value.values:
        assert callable(getattr(TaskRunnerNode, handler.attr))


@pytest.mark.parametrize('path', sorted((ROOT / 'config/tasks').glob('*.yaml')),
                         ids=lambda path: path.name)
def test_bundled_task_configurations_load(path):
    tasks = load_mission_or_task(path)
    assert len(tasks) == 1
    assert tasks[0]['name']


class TaskNode:
    """Mock subscriptions while checking the installed rclpy method signature."""
    stopped = False
    LIGHT_OFF = 0
    LIGHT_RED = 1
    LIGHT_GREEN = 2
    LIGHT_YELLOW = 3
    LIGHT_BLUE = 4

    def __init__(self):
        self.camera_configs = {
            camera: load_camera_config(camera, 'sim') for camera in ('front', 'down')}
        optical = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
        self.camera_extrinsics = {
            eye: NS(translation=np.zeros(3), optical_to_body=optical)
            for eye in ('front_left', 'front_right', 'down_left', 'down_right')}
        self._model_mapping = NS(
            model_class_id=lambda name, required=False: 7,
            configured_class_id=lambda params, key, name, required=False: 7)
        self._perception_lock = threading.RLock()
        self._down_detection_sequence = 0
        self.subscriptions = []

    def get_logger(self):
        return get_logger('task_compatibility')

    def create_subscription(self, *args, **kwargs):
        inspect.signature(Node.create_subscription).bind(self, *args, **kwargs)
        subscription = object()
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        self.subscriptions.remove(subscription)

    def get_clock(self):
        return NS(now=lambda: NS(nanoseconds=100_000_000_000))

    def _latest_robot_pose(self):
        return (0., 0., .2, 0., 0., 0.)


TASK_CLASSES = [
    ('26rb_gate_task', 'RB26GateTask'),
    ('26rb_hit_balls', 'RB26HitBallsTask'),
    ('26rb_grab_golf', 'RB26GrabGolfTask'),
    ('26rb_find_collection_frame', 'RB26FindCollectionFrameTask'),
    ('26rb_drop_ball_target_rack', 'RB26DropBallTargetRackTask'),
    ('26rb_drop_beacon', 'RB26DropBeaconTask'),
    ('line_follower', 'LineFollower'),
    ('arrow_surfacer', 'ArrowSurfacer'),
    ('collection_frame_search', 'CollectionFrameSearch'),
    ('collection_frame_search', 'TargetRackSearch'),
]


@pytest.mark.parametrize('module_name,class_name', TASK_CLASSES)
def test_task_constructors(module_name, class_name):
    cls = getattr(importlib.import_module('uv_task.' + module_name), class_name)
    task = cls(TaskNode(), {})
    if hasattr(task, 'destroy'):
        task.destroy()


@pytest.mark.parametrize('kind', ['observer', 'collection_frame', 'target_rack'])
@pytest.mark.parametrize('eye', ['left', 'right'])
def test_front_detection_callback_accepts_each_camera(kind, eye):
    node = TaskNode()
    if kind == 'observer':
        from uv_task.front_target_observer import FrontTargetObserver
        task = FrontTargetObserver(node, HIT_DEFAULTS)
        task.now = lambda: 100.
        task.select(7)
        callback, latest = task._callback, task.latest
    else:
        from uv_task.collection_frame_search import CollectionFrameSearch, TargetRackSearch
        cls = CollectionFrameSearch if kind == 'collection_frame' else TargetRackSearch
        task = cls(node, {})
        task._now = lambda: 100.
        callback, latest = task._detection_cb, task._latest
    calibration = node.camera_configs['front'].side(eye)
    x, y = calibration.matrix[0, 2], calibration.matrix[1, 2]
    camera = 'front_' + eye
    detection = NS(class_id=7, confidence=.8, pixel_x=x, pixel_y=y,
                   bbox_x1=x-10, bbox_y1=y-10, bbox_x2=x+10, bbox_y2=y+10)
    callback(NS(camera_name=camera, detections=[detection],
                header=NS(stamp=NS(sec=100, nanosec=0)), stereo_pair_id=1))
    assert latest[camera] is not None
    assert np.isclose(np.linalg.norm(latest[camera].ray), 1.)


def test_task_launch_description_loads():
    import runpy
    from launch import LaunchDescription

    launch = runpy.run_path(str(ROOT / 'launch/task_launch.py'))
    assert isinstance(launch['generate_launch_description'](), LaunchDescription)
