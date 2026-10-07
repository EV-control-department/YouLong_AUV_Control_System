"""Hardware forwarding and single-flight async service behavior."""

import threading
from types import SimpleNamespace

import pytest
from rclpy.task import Future
from std_msgs.msg import UInt32
from zit6_interfaces.srv import GetParams, SetOrigin, UpdateParams
from uv_hm.hw_manager import HwManagerNode
import uv_hm.hw_manager as module


class Client:
    def __init__(self):
        self.calls = []

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        result = Future()
        self.calls.append(result)
        return result

    def remove_pending_request(self, _future):
        pass


@pytest.fixture
def manager(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    node = HwManagerNode.__new__(HwManagerNode)
    node._Node__executor_weakref = None
    node._origin_proxy_lock = threading.Lock()
    node._pending_origin_proxy = None
    node._param_proxy_lock = threading.Lock()
    node._pending_param_proxies = {}
    node._param_clients = {'get': Client(), 'update': Client()}
    node._origin_client = Client()
    node.get_parameter = lambda _name: SimpleNamespace(value=2.0)
    return node, clock


def advance(coroutine):
    with pytest.raises(StopIteration) as stop:
        coroutine.send(None)
    return stop.value.value


def test_heartbeat_forwarding_preserves_owner_command_without_generation(manager):
    node, _clock = manager
    messages = []
    node._heartbeat_pub = SimpleNamespace(publish=messages.append)
    for mode in (0, 1, 3):
        message = UInt32(data=mode)
        node._heartbeat_cb(message)
        assert messages[-1] is message
    assert [message.data for message in messages] == [0, 1, 3]


def test_async_proxy_preserves_mcu_response_and_rejects_overlap(manager):
    node, _clock = manager
    call = node._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    call.send(None)
    overlap = node._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    rejected = advance(overlap)
    assert not rejected.success and rejected.message == 'setorigin busy'
    response = SetOrigin.Response(success=True, message='origin set',
                                  origin_nav=[1.0, 2.0, 3.0, 0.0, 0.0, 0.5],
                                  nav_timestamp_ms=123, origin_generation=9)
    node._origin_client.calls[0].set_result(response)
    adopted = advance(call)
    assert adopted.success and list(adopted.origin_nav) == list(response.origin_nav)
    assert adopted.nav_timestamp_ms == 123 and adopted.origin_generation == 9
    assert len(node._origin_client.calls) == 1


def test_async_proxy_times_out_and_late_response_cannot_complete_new_call(manager):
    node, clock = manager
    call = node._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    call.send(None)
    pending = node._pending_origin_proxy
    clock[0] += 2.1
    node._origin_timeout_cb()
    failed = advance(call)
    assert not failed.success and 'timeout' in failed.message
    newer = node._set_origin_cb(SetOrigin.Request(), SetOrigin.Response())
    newer.send(None)
    late = Future()
    late.set_result(SetOrigin.Response(success=True))
    node._origin_proxy_done(pending, late)
    assert not node._pending_origin_proxy['completion'].done()
    node._origin_client.calls[-1].set_result(SetOrigin.Response(success=False, message='armed'))
    assert advance(newer).message == 'armed'


@pytest.mark.parametrize('name, service', [('get', GetParams), ('update', UpdateParams)])
def test_parameter_proxy_preserves_response_and_is_single_flight(manager, name, service):
    node, _clock = manager
    call = node._proxy_params(name, service.Request(), service.Response())
    call.send(None)
    overlap = node._proxy_params(name, service.Request(), service.Response())
    assert not advance(overlap).success
    result = service.Response(success=True, message='updated')
    if name == 'get':
        result.config_json = '{"simulation":{"sitl_enabled":true}}'
    node._param_clients[name].calls[0].set_result(result)
    adopted = advance(call)
    assert adopted.success and adopted.message == 'updated'
    if name == 'get':
        assert adopted.config_json == result.config_json


def test_parameter_timeout_ignores_late_completion(manager):
    node, clock = manager
    call = node._proxy_params('update', UpdateParams.Request(), UpdateParams.Response())
    call.send(None)
    pending = node._pending_param_proxies['update']
    clock[0] += 2.1
    node._params_timeout_cb()
    assert not advance(call).success
    newer = node._proxy_params('update', UpdateParams.Request(), UpdateParams.Response())
    newer.send(None)
    late = Future()
    late.set_result(UpdateParams.Response(success=True))
    node._params_proxy_done('update', pending, late)
    assert not node._pending_param_proxies['update']['completion'].done()
    node._param_clients['update'].calls[-1].set_result(UpdateParams.Response(success=True))
    assert advance(newer).success
