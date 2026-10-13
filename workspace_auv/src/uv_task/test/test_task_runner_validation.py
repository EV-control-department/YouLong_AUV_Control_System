"""``uv_task`` 的第一组测试：任务清单的校验与取消 API 的用法。

This package had **no tests at all** until now, and the two defects fixed
2026-10-12 are exactly the kind that a test would have caught:

* **task-name validation.** ``run_task_list`` deliberately does *not* abort on a
  failed task, so an unknown ``name`` used to produce one ``warn`` at execution
  time and then the AUV would silently skip a step -- with every later command
  running from a stale ``_cmd_*`` estimate.  ``load_tasks`` now validates, so a
  typo is caught at startup instead.
* **cancelling a goal.** ``/task/stop`` called
  ``self._action_client.async_cancel_goal(handle)``, but rclpy's ``ActionClient``
  has no such method (and no ``cancel_goal_async`` either) -- cancellation lives
  on the **goal handle**.  The ``AttributeError`` was swallowed by the service
  callback, so ``/task/stop`` never actually stopped the running goal.
"""

from __future__ import annotations

import inspect
import json
import re
import threading
from pathlib import Path

import pytest
import rclpy

from rclpy.action import ActionClient

TASK_RUNNER_PY = Path(__file__).resolve().parents[1] / 'uv_task' / 'task_runner.py'


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def node(ros_context):
    """一个模块共用一个节点。

    ``TaskRunnerNode.__init__`` 里有 ``wait_for_server(timeout_sec=5.0)``（等
    ``basic_motion``），测试环境里那个 server 不存在，所以**每新建一个节点就白等
    5 秒** —— 逐测试新建会让这个文件跑到 55 s。``validate_tasks`` 不依赖节点状态，
    共用一个即可。
    """
    from uv_task.task_runner import TaskRunnerNode

    n = TaskRunnerNode()
    yield n
    n.destroy_node()


# --------------------------------------------------------------------------- #
# 取消 API 的用法（2026-10-12 修复的那个 bug）
# --------------------------------------------------------------------------- #
def test_action_client_has_no_cancel_method() -> None:
    """钉住这个前提：取消**不可能**在 ActionClient 上做。

    如果哪天 rclpy 加了客户端侧的取消方法，这条会失败并提醒我们重新评估
    task_runner 里的两处调用。
    """
    for name in ('async_cancel_goal', 'cancel_goal_async', 'cancel_goal'):
        assert not hasattr(ActionClient, name), (
            f'rclpy 现在有 ActionClient.{name} 了 —— task_runner 的取消写法需要复核'
        )


def test_cancellation_uses_the_goal_handle_not_the_action_client() -> None:
    """静态检查：所有取消都必须走 goal handle。"""
    source = TASK_RUNNER_PY.read_text(encoding='utf-8')
    # 允许出现在注释里（解释为什么不能这么写），但不允许出现在真实调用里
    code_lines = [
        ln for ln in source.splitlines()
        if not ln.lstrip().startswith('#')
    ]
    code = '\n'.join(code_lines)
    assert '_action_client.async_cancel_goal' not in code, (
        'in ActionClient 上没有 async_cancel_goal；取消要用 handle.cancel_goal_async()'
    )
    assert 'handle.cancel_goal_async()' in source, (
        '取消必须在 goal handle 上调用 cancel_goal_async()'
    )


# --------------------------------------------------------------------------- #
# 任务清单校验
# --------------------------------------------------------------------------- #
def test_valid_task_list_has_no_problems(node) -> None:
    problems = node.validate_tasks(
        [{'name': 'start', 'params': {}},
         {'name': 'setz', 'params': {'z': 1.0}},
         {'name': 'bvelocity', 'params': {'vx': 0.4, 'duration': 3.0}}],
        'ok.json')
    assert problems == []


def test_typo_in_a_task_name_is_reported_with_the_valid_names(node) -> None:
    """最有价值的一条：打错的任务名必须在启动时就报出来。

    放在列表中间也一样 —— 正是"前面的任务都成功了、第 7 条打错字"这种情况
    以前最难发现（失败不中止列表）。
    """
    problems = node.validate_tasks(
        [{'name': 'start'}, {'name': 'setzz', 'params': {'z': 1.0}}], 'x.json')
    assert len(problems) == 1
    assert 'setzz' in problems[0]
    assert 'tasks[1]' in problems[0], "要指出是第几条"
    # 报错要附带可用名字，否则使用者还得去翻源码
    assert 'setz' in problems[0] and 'bmovex' in problems[0]


@pytest.mark.parametrize("bad,expect", [
    ([{'name': 'start'}, {'params': {}}], 'name'),
    ([{'name': ''}], 'name'),
    ([{'name': 123}], 'name'),
    ([{'name': 'start', 'params': [1, 2]}], 'params'),
    ([{'name': 'start'}, 'not-a-dict'], '不是对象'),
    ({'tasks': 'nope'}, '数组'),
])
def test_malformed_entries_are_reported(node, bad, expect) -> None:
    problems = node.validate_tasks(bad, 'x.json')
    assert problems, f'{bad!r} 应该被判为非法'
    assert any(expect in p for p in problems), problems


def test_load_tasks_never_raises_and_returns_empty_on_bad_input(node, tmp_path) -> None:
    """坏输入必须返回 []，不能抛。

    这是服务回调路径：服务回调里抛异常 = 客户端拿不到响应、一直挂到超时，
    而且看不出原因。
    """
    cases = {
        'invalid_json.json': '{bad json',
        'root_is_list.json': '[1, 2, 3]',
        'missing_tasks.json': '{"foo": 1}',
        'bad_name.json': json.dumps({'tasks': [{'name': 'nope'}]}),
    }
    for filename, content in cases.items():
        path = tmp_path / filename
        path.write_text(content, encoding='utf-8')
        assert node.load_tasks(str(path)) == [], f'{filename} 应返回 []'

    assert node.load_tasks(str(tmp_path / 'does_not_exist.json')) == []


def test_load_tasks_accepts_a_valid_file(node, tmp_path) -> None:
    path = tmp_path / 'good.json'
    path.write_text(json.dumps({'tasks': [
        {'name': 'start'}, {'name': 'stay'},
    ]}), encoding='utf-8')
    tasks = node.load_tasks(str(path))
    assert [t['name'] for t in tasks] == ['start', 'stay']


def test_default_task_list_validates(node) -> None:
    """仓库里自带的 tasks.json 自身必须能过校验。

    否则每次正常启动都会直接拒绝加载 —— 这个测试就是防它腐烂。
    """
    from ament_index_python.packages import get_package_share_directory
    import os

    path = os.path.join(get_package_share_directory('uv_task'),
                        'config', 'tasks.json')
    with open(path, encoding='utf-8') as f:
        data = json.load(f)
    problems = node.validate_tasks(data['tasks'], path)
    assert problems == [], f'仓库自带的 tasks.json 校验失败: {problems}'


# --------------------------------------------------------------------------- #
# R4：goal 已发出但句柄尚未记录时收到 /task/stop
# --------------------------------------------------------------------------- #
class _FakeFuture:
    """只实现 `_send_action_goal` 用到的那两个方法：done() / result()。"""

    def __init__(self, value=None, already_done=False):
        self._value = value
        self._done = threading.Event()
        if already_done:
            self._done.set()

    def set_result(self, value):
        self._value = value
        self._done.set()

    def done(self):
        return self._done.is_set()

    def result(self):
        return self._value


class _FakeResult:
    def __init__(self, success=True, message=''):
        self.success = success
        self.message = message


class _FakeResultWrapper:
    def __init__(self, result):
        self.result = result


class _FakeGoalHandle:
    def __init__(self, accepted=True, result_future=None):
        self.accepted = accepted
        self.cancel_calls = 0
        self._result_future = result_future or _FakeFuture()   # 默认永不完成

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _FakeFuture()

    def get_result_async(self):
        return self._result_future


class _FakeActionClient:
    def __init__(self, send_future):
        self._send_future = send_future
        self.sent = []

    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal):
        self.sent.append(goal)
        return self._send_future


@pytest.fixture
def fake_client(node):
    """替换 node._action_client，测试后还原（node 是模块级共享的）。"""
    original = node._action_client
    holder = {}
    yield holder
    node._action_client = original
    node.stopped = False
    node._debug_timeout = -1.0
    node._active_goal_handle = None


def test_stop_arriving_while_the_goal_is_in_flight_still_cancels(node, fake_client):
    """R4：句柄还没回来时 stop 先到 —— goal 仍必须被取消。

    原实现在 `stopped` 分支直接 return，那时 `_active_goal_handle` 还是 None
    （句柄要等 goal response 才拿得到），于是 `/task/stop` 找不到它、代码也不再
    取消它 —— goal 在服务端继续跑到目标。实测那次 `basic_motion` 的
    `等待到达` 一路继续了十几秒。
    """
    import time as _time
    from uv_msgs.action import BasicMotion

    send_future = _FakeFuture()
    handle = _FakeGoalHandle()
    node._action_client = _FakeActionClient(send_future)
    node.stopped = False
    node._debug_timeout = -1.0

    out = {}

    def worker():
        out['ret'] = node._send_action_goal(
            BasicMotion.Goal.SET, [1.0, 0.0, 0.0, 0.0], timeout=5.0, quiet=True)

    thread = threading.Thread(target=worker, daemon=True)
    thread.start()

    # 等到 goal 真的发出去了（这就是竞态窗口：已发出、句柄未记录）
    deadline = _time.monotonic() + 3.0
    while not node._action_client.sent and _time.monotonic() < deadline:
        _time.sleep(0.01)
    assert node._action_client.sent, 'send_goal_async 没被调用'
    assert node._active_goal_handle is None, '此刻句柄本应还没记录'

    node.stopped = True                 # /task/stop 到达
    send_future.set_result(handle)      # 响应对回来了

    thread.join(timeout=5.0)
    assert not thread.is_alive()
    assert out['ret'] == (False, 'Stopped'), out['ret']
    assert handle.cancel_calls == 1, (
        'goal 已经发出去却没有被取消 —— 它会继续在服务端跑到目标')


def test_a_successful_goal_is_not_cancelled(node, fake_client):
    """对照组：正常完成的 goal 不能被误取消。"""
    from uv_msgs.action import BasicMotion

    handle = _FakeGoalHandle(
        result_future=_FakeFuture(_FakeResultWrapper(_FakeResult(True, '')), already_done=True))
    node._action_client = _FakeActionClient(_FakeFuture(handle, already_done=True))
    node.stopped = False
    node._debug_timeout = -1.0

    ok, msg = node._send_action_goal(
        BasicMotion.Goal.SET, [1.0, 0.0, 0.0, 0.0], timeout=5.0, quiet=True)
    assert (ok, msg) == (True, ''), (ok, msg)
    assert handle.cancel_calls == 0, '成功的 goal 不该被取消'
    assert node._active_goal_handle is None, '结束后句柄应清空'
