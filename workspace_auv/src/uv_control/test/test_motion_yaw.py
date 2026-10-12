"""Exercise real step planning without ROS nodes or hardware commands."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from uv_control.coordinate import Coordinate, wrap_deg


def motion_method(name):
    source = Path(__file__).parents[1] / 'uv_control' / 'basic_motion.py'
    cls = next(node for node in ast.parse(source.read_text()).body
               if isinstance(node, ast.ClassDef) and node.name == 'BasicMotionNode')
    method = next(node for node in cls.body
                  if isinstance(node, ast.FunctionDef) and node.name == name)
    scope = dict(math=math, Coordinate=Coordinate, wrap_deg=wrap_deg,
                 time=SimpleNamespace(monotonic=lambda: 0.0),
                 rclpy=SimpleNamespace(ok=lambda: True), LATERAL_LAMBDA=1.0, TOL_X=.1, TOL_Y=.1)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    return scope[name]


@pytest.mark.parametrize('pose_yaw,target_yaw,goal_yaw,expected_increment', [
    (179., 179., -179., 1.),
    (-179., -179., 179., -1.),
    (179., -179., -179., 0.),
    (-179., 179., 179., 0.),
    (80., 90., 90., 0.),
    (-80., -90., -90., 0.),
    (170., 170., 190., 10.),
    (-170., -170., -190., -10.),
    (180., 180., -180., 0.),
    (-180., -180., 180., 0.),
])
def test_step_yaw_uses_shortest_arc_from_commanded_target(
        pose_yaw, target_yaw, goal_yaw, expected_increment):
    pose = Coordinate(rz=pose_yaw)
    target = Coordinate(rz=target_yaw)
    commands = []
    logger = SimpleNamespace(info=lambda msg: None, warning=lambda msg: None)
    node = SimpleNamespace(
        get_state=lambda: (pose, target, None), get_logger=lambda: logger,
        _is_cancelled=lambda: False, _calc_step_size=lambda angle: 1.,
        _odom_to_body=lambda point: pose.world_to_body(point.x-pose.x, point.y-pose.y),
        _wait_step_convergence=lambda angle, timeout: True,
        _cmd_and_wait=lambda *args, **kwargs: True)

    def set_world(x, y, z, yaw):
        target.x, target.y, target.z, target.rz = x, y, z, yaw

    node.set_world = set_world
    actual_step = motion_method('set_step')

    def step(**kwargs):
        commands.append(kwargs['dyaw_deg'])
        actual_step(node, **kwargs)
        pose.x = 1.5  # Finish translation so this exercises exactly one planned step.

    node.set_step = step
    assert motion_method('_step_move_world')(node, 1.5, 0., 0., goal_yaw, timeout=10.)
    assert commands == pytest.approx([expected_increment])
    assert wrap_deg(target.rz-goal_yaw) == pytest.approx(-expected_increment)


def test_travel_keeps_direction_across_negative_x_axis():
    calls = []
    logger = SimpleNamespace(info=lambda msg: None, warning=lambda msg: None)
    node = SimpleNamespace(get_state=lambda: (Coordinate(rz=179.), Coordinate(), None),
                           get_logger=lambda: logger)
    node._step_move_world = lambda *args, **kwargs: calls.append(args) or True
    assert motion_method('_travel_world')(node, -2., -0.01, 0.4, 90., timeout=10.)
    direction = math.degrees(math.atan2(-0.01, -2.))
    assert len(calls) == 2
    assert calls[0][3] == pytest.approx(direction)
    assert calls[1][3] == pytest.approx(direction)
    assert abs(wrap_deg(direction-179.)) < 2.


@pytest.mark.parametrize('xy', [(0.,.03), (-.09,0.), (.09,.09)])
def test_travel_near_point_keeps_requested_yaw_instead_of_noise_bearing(xy):
    calls = []
    node = SimpleNamespace(get_state=lambda: (Coordinate(z=.4, rz=90.), Coordinate(), None),
        get_logger=lambda: SimpleNamespace(info=lambda s: None, warning=lambda s: None))
    node._step_move_world = lambda *args, **kwargs: calls.append(args) or True
    assert motion_method('_travel_world')(node, xy[0], xy[1], .4, 90., timeout=10.)
    assert len(calls) == 1
    assert calls[0][3] == 90.
