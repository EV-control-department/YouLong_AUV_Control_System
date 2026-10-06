"""A zero target after START must hold the current submerged map pose."""

import ast
from pathlib import Path
import threading
from types import SimpleNamespace as NS

import pytest

from uv_control.coordinate import Coordinate


def start_method():
    source = Path(__file__).parents[1] / 'uv_control/basic_motion.py'
    tree = ast.parse(source.read_text())
    cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'BasicMotionNode')
    method = next(item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == 'start')
    namespace = {'Coordinate': Coordinate, 'Empty': lambda: object()}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    return namespace['start']


@pytest.mark.parametrize('origin,pose,expected', [
    (Coordinate(z=0.0), Coordinate(z=1.2), [0.0, 0.0, 1.2, 0.0]),
    (Coordinate(x=10, y=20, z=3, rz=90), Coordinate(x=2, y=1, z=1, rz=40), [9.0, 22.0, 4.0, 130.0]),
])
def test_start_accounts_for_current_position_since_estimator_origin(origin, pose, expected):
    logger = NS(info=lambda text: None)
    node = NS(_origin=None, _state_origin=origin, pose=pose,
              _state_lock=threading.Lock(), _sim_mode=False,
              get_logger=lambda: logger, _publish_while_running=lambda *args: None,
              pub_state_reset=object())
    start_method()(node)
    target = node._origin.to_world_frame(Coordinate())
    assert [target.x, target.y, target.z, target.rz] == pytest.approx(expected)
    assert [node.pose.x, node.pose.y, node.pose.z, node.pose.rz] == [0.0] * 4


def test_sim_estimator_reset_keeps_zero_map_origin():
    logger = NS(info=lambda text: None)
    node = NS(_origin=Coordinate(), _state_origin=Coordinate(),
              pose=Coordinate(x=2, z=1), _state_lock=threading.Lock(), _sim_mode=True,
              get_logger=lambda: logger, _publish_while_running=lambda *args: None,
              pub_state_reset=object())
    start_method()(node)
    assert [node._origin.x, node._origin.z] == [0.0, 0.0]


@pytest.mark.parametrize('target', [[0.0] * 4, [0.0]])
def test_velocity_goal_preserves_cancelled_position_handle(target):
    source = Path(__file__).parents[1] / 'uv_control/basic_motion.py'
    cls = next(item for item in ast.parse(source.read_text()).body
               if isinstance(item, ast.ClassDef) and item.name == 'BasicMotionNode')
    method = next(item for item in cls.body
                  if isinstance(item, ast.FunctionDef) and item.name == '_action_execute_cb')
    namespace = {'BasicMotion': NS(Goal=NS(START=6, BODY_VELOCITY=7), Result=NS),
                 'DEFAULT_VELOCITY_LEASE': 0.25}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    active = NS(is_cancel_requested=True)
    node = NS(_action_goal_handle=active, _publish_body_velocity=lambda *args, **kw: None)
    goal = NS(request=NS(cmd_type=7, target=target, velocity_lease=0.25),
              succeed=lambda: None, abort=lambda: None)
    namespace['_action_execute_cb'](node, goal)
    assert node._action_goal_handle is active
    assert node._action_goal_handle.is_cancel_requested
