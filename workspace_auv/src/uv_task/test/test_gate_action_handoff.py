"""Exercise the runner's production waits without importing/starting its node."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS

import pytest
from uv_msgs.action import BasicMotion
from uv_task.task_outcome import TaskOutcome

ROOT = Path(__file__).parents[1]


def method(name, namespace):
    path = ROOT/'uv_task/task_runner.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'TaskRunnerNode')
    function = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class Future:
    def __init__(self, value=None):
        self.value = value
        self.callbacks = []

    def done(self):
        return self.value is not None

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.done():
            callback(self)

    def resolve(self, value):
        self.value = value
        for callback in self.callbacks:
            callback(self)


@pytest.mark.parametrize('late_acceptance', [False, True])
def test_action_deadline_cancels_current_or_late_goal(late_acceptance):
    now = [0.]
    namespace = {'BasicMotion': BasicMotion,
                 'rclpy': NS(ok=lambda: True),
                 'time': NS(monotonic=lambda: now[0], sleep=lambda dt: now.__setitem__(0, now[0]+dt))}
    send = method('_send_action_goal', namespace)
    cancelled = []
    handle = NS(accepted=True, get_result_async=lambda: Future(),
                cancel_goal_async=lambda: cancelled.append(True))
    acknowledgement = Future(None if late_acceptance else handle)
    logger = NS(info=lambda *_: None, error=lambda *_: None, warn=lambda *_: None)
    node = NS(stopped=False, _debug_timeout=0., _active_goal_handle=None,
              _action_client=NS(wait_for_server=lambda **_: True,
                                send_goal_async=lambda _: acknowledgement),
              _set_task_phase_light=lambda *a, **k: None,
              LIGHT_YELLOW=1, get_logger=lambda: logger)
    success, _ = send(node, BasicMotion.Goal.BLINE, [1.6, 0., 0., 0.],
                      axes='xyz', timeout=60., wait_deadline=.1,
                      task_context='gate acceptance', quiet=True)
    assert not success
    assert now[0] < .12
    assert node._last_motion_failure_kind == 'timeout'
    if late_acceptance:
        assert not cancelled
        acknowledgement.resolve(handle)
    assert cancelled == [True]


def test_task_runner_preserves_gate_failure_flash_and_resets_for_next_task():
    execute = method('_execute_task', {'TaskOutcome': TaskOutcome})
    lights = []
    node = NS(LIGHT_RED=3, _set_task_phase_light=lambda color, *a: lights.append(color))
    def gate(_):
        node._task_failure_light_handled = True
        return TaskOutcome.failed('26rb_gate_task.search', 'red double blink already sent')
    node.task_map = {'26rb_gate_task': gate, 'next': lambda _: TaskOutcome.failed('next.motion')}
    assert not execute(node, '26rb_gate_task', {})
    assert lights == []
    assert not execute(node, 'next', {})
    assert lights == [3]
