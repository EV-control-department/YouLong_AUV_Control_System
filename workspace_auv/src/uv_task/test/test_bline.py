"""BLINE task configuration and returned command-pose synchronization."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from uv_task.config_loader import load_mission_or_task


ROOT = Path(__file__).parents[1]


def test_bline_default_yaml():
    tasks = load_mission_or_task(ROOT / 'config/tasks/bline.yaml')
    assert tasks[0]['name'] == 'bline'
    assert tasks[0]['params'] == dict(dx=1.0, dy=0.0, dz=0.0,
                                      drz=0., speed_mps=.15, timeout=0.0)


@pytest.mark.parametrize('target,success', [
    ([4.0, -2.0, .3, 90.0], True),
    (None, False), ([0.0, 0.0, 0.0], False), ([0, 0, math.nan, 0], False),
])
def test_command_pose_uses_returned_fixed_target(target, success):
    source = ROOT / 'uv_task/task_runner.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TaskRunnerNode')
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_task_bline')
    namespace = dict(math=math, BasicMotion=NS(Goal=NS(BLINE=8)))
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), namespace)
    calls = []
    logger = NS(info=lambda *_: None, error=lambda *_: None)
    runner = NS(_last_motion_final_target=target,
                _cmd_x=100, _cmd_y=100, _cmd_z=100, _cmd_yaw=180,
                get_logger=lambda: logger)
    runner._send_action_goal = lambda *args, **kwargs: (
        calls.append((args, kwargs)) or (True, ''))
    assert namespace['_task_bline'](runner, dict(dx=0, dy=1, dz=.3)) == success
    assert calls == [((8, [0.0, 1.0, .3, 0.0], 'xyz'),
                      dict(timeout=0.0, cruise_speed=.15))]
    assert (runner._cmd_x, runner._cmd_y, runner._cmd_z, runner._cmd_yaw) == (
        tuple(target) if success else (100, 100, 100, 180))


def test_wline_default_yaml_and_body_final_rotation():
    tasks = load_mission_or_task(ROOT / 'config/tasks/wline.yaml')
    assert tasks[0]['name'] == 'wline'
    assert tasks[0]['params'] == dict(x=1., y=0., z=0., rz=0., speed_mps=.15, timeout=0.)
    tasks = load_mission_or_task(ROOT / 'config/tasks/bline.yaml')
    assert tasks[0]['params']['drz'] == 0.


@pytest.mark.parametrize('name,params,command,target', [
    ('_task_bline', dict(dx=1,dy=2,dz=.3,drz=45), 8, [1.,2.,.3,45.]),
    ('_task_wline', dict(x=1,y=2,z=.3,rz=-45), 9, [1.,2.,.3,-45.]),
])
def test_task_forwards_final_heading_and_syncs_returned_world_pose(name, params, command, target):
    source = ROOT / 'uv_task/task_runner.py'
    cls = next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='TaskRunnerNode')
    method = next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    ns = dict(math=math, BasicMotion=NS(Goal=NS(BLINE=8,WLINE=9)))
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),ns)
    calls=[]
    runner=NS(_last_motion_final_target=[4.,5.,.3,135.],
              _cmd_x=0.,_cmd_y=0.,_cmd_z=0.,_cmd_yaw=0.,
              get_logger=lambda:NS(info=lambda text:None,error=lambda text:None))
    runner._send_action_goal=lambda *args,**kw: calls.append((args,kw)) or (True,'')
    assert ns[name](runner,params)
    assert calls[0][0] == (command,target,'xyz')
    assert (runner._cmd_x,runner._cmd_y,runner._cmd_z,runner._cmd_yaw)==(4.,5.,.3,135.)
