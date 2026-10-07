"""START服务握手回归测试；不创建节点、不发布指令、不访问真机。"""
import ast
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace


def reset_method():
    source = Path(__file__).parents[1] / 'uv_control' / 'basic_motion.py'
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BasicMotionNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_reset_mcu_origin')
    scope = {'math': math, 'time': time, 'rclpy': SimpleNamespace(ok=lambda: True),
             'SetOrigin': SimpleNamespace(Request=lambda: object())}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    return scope['_reset_mcu_origin']


def fake_node(response=None, generation=1):
    calls = []
    client = SimpleNamespace(wait_for_service=lambda **kwargs: True,
                             call_async=lambda req: calls.append(req) or SimpleNamespace(
                                 done=lambda: True, result=lambda: response))
    node = SimpleNamespace(
        _origin_client=client, _state_lock=threading.Lock(), _origin=object(),
        _mcu_odom=SimpleNamespace(origin_initialized=True, nav_valid=True, origin_generation=generation),
        _mcu_odom_received=time.monotonic(),
        get_parameter=lambda name: SimpleNamespace(value=.03),
        get_logger=lambda: SimpleNamespace(info=lambda msg: None))
    return node, calls


def test_success_requires_matching_generation():
    node, calls = fake_node(SimpleNamespace(success=True, origin_generation=2), generation=2)
    success, message = reset_method()(node, SimpleNamespace(is_cancel_requested=False))
    assert success and 'generation=2' in message
    assert len(calls) == 1 and node._origin is None


def test_service_rejection_is_reported_without_retry():
    node, calls = fake_node(SimpleNamespace(success=False, message='navigation invalid or stale'))
    success, message = reset_method()(node, SimpleNamespace(is_cancel_requested=False))
    assert not success and 'navigation invalid or stale' in message
    assert len(calls) == 1


def test_old_generation_does_not_confirm_reset():
    node, calls = fake_node(SimpleNamespace(success=True, origin_generation=2), generation=1)
    success, message = reset_method()(node, SimpleNamespace(is_cancel_requested=False))
    assert not success and '未收到有效同代odom' in message
    assert len(calls) == 1


def test_missing_service_and_cancel_do_not_send_reset():
    node, calls = fake_node()
    node._origin_client.wait_for_service = lambda **kwargs: False
    assert not reset_method()(node, SimpleNamespace(is_cancel_requested=False))[0]
    assert not calls
    node._origin_client.wait_for_service = lambda **kwargs: True
    assert not reset_method()(node, SimpleNamespace(is_cancel_requested=True))[0]
    assert not calls


def test_response_timeout_is_not_retried():
    node, calls = fake_node()
    canceled = []
    node._origin_client.call_async = lambda req: calls.append(req) or SimpleNamespace(
        done=lambda: False, cancel=lambda: canceled.append(True))
    success, message = reset_method()(node, SimpleNamespace(is_cancel_requested=False))
    assert not success and '请求可能已执行' in message
    assert len(calls) == len(canceled) == 1


def extract_method(name, **extra):
    source = Path(__file__).parents[1] / 'uv_control' / 'basic_motion.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'BasicMotionNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    scope = {'time': time, 'rclpy': SimpleNamespace(ok=lambda: True), **extra}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    return scope[name]


def test_prepare_start_allows_other_publishers_when_mcu_is_disarmed():
    node, _ = fake_node()
    node._heartbeat_enabled = True
    node.count_publishers = lambda topic: 2
    node._last_status_received = time.monotonic()
    node.status = SimpleNamespace(is_armed=False)
    success, message = extract_method('_prepare_start')(node, SimpleNamespace(is_cancel_requested=False))
    assert success and message == 'MCU已上锁'
    assert not node._heartbeat_enabled and node._origin is None


def test_prepare_start_requires_fresh_disarmed_status():
    node, _ = fake_node()
    node.count_publishers = lambda topic: 1
    node._last_status_received = time.monotonic()
    node.status = SimpleNamespace(is_armed=False)
    assert extract_method('_prepare_start')(node, SimpleNamespace(is_cancel_requested=False))[0]
    assert not extract_method('_prepare_start')(node, SimpleNamespace(is_cancel_requested=True))[0]


def test_heartbeat_only_publishes_when_start_enabled_it():
    node, _ = fake_node()
    packets = []
    node._heartbeat_pub = SimpleNamespace(publish=lambda msg: packets.append(msg.data))
    node.get_parameter = lambda name: SimpleNamespace(value=1)
    method = extract_method('_heartbeat_cb', UInt32=lambda **kw: SimpleNamespace(**kw))
    node._heartbeat_enabled = False
    method(node)
    assert packets == []
    node._heartbeat_enabled = True
    method(node)
    assert packets == [1]


def test_start_publishes_pose_before_enabling_heartbeat():
    events = []
    node = SimpleNamespace(
        _use_mcu_odom=True, _state_lock=threading.Lock(),
        _prepare_start=lambda goal: (events.append('disarm') or True, ''),
        _reset_mcu_origin=lambda goal: (events.append('reset') or True, ''),
        start=lambda: events.append('local_origin'),
        _publish_pose_info=lambda: events.append('pose'),
        _heartbeat_cb=lambda: events.append('heartbeat'),
        get_logger=lambda: SimpleNamespace(info=lambda msg: None))
    goal = SimpleNamespace(request=SimpleNamespace(cmd_type=6, task_context='test'),
                           succeed=lambda: events.append('success'))
    action = SimpleNamespace(Goal=SimpleNamespace(START=6), Result=lambda: SimpleNamespace())
    result = extract_method('_action_execute_cb', BasicMotion=action)(node, goal)
    assert result.success
    assert events == ['disarm', 'reset', 'local_origin', 'pose', 'heartbeat', 'success']


def test_idle_diagnostic_explains_missing_odom_and_disabled_heartbeat():
    logs = []
    node, _ = fake_node()
    node._diagnostic_last_log = float('-inf')
    node._mcu_odom = None
    node._last_status_received = 0.
    node.status = SimpleNamespace(is_armed=False, navigation_ready=False)
    node._odom_received_count = node._heartbeat_sent = 0
    node._heartbeat_enabled = False
    node.get_logger = lambda: SimpleNamespace(info=logs.append)
    method = extract_method('_diagnostic_cb')
    method(node)
    method(node)
    assert len(logs) == 1  # 高频重复调用也不得刷屏。
    assert '未收到有效格式odom' in logs[0]
    assert '心跳启用=False，已发送=0包' in logs[0]
