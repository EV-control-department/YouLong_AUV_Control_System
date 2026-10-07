"""State and reset behavior with real ROS messages and a deterministic clock."""

from collections import OrderedDict
import math
import threading
from types import SimpleNamespace

import pytest
from rclpy.task import Future
from uv_msgs.msg import StateResetRequest
from zit6_interfaces.msg import ZitOdom
from zit6_interfaces.srv import SetOrigin
from uv_localization.estimator import EstimatorNode, _at_least_u32
import uv_localization.estimator as module


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Client:
    def __init__(self):
        self.futures = []
        self.removed = []

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        future = Future()
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


@pytest.fixture
def adapter(monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    node = EstimatorNode.__new__(EstimatorNode)
    node._lock = threading.Lock()
    node._odom = None
    node._last_odom_time = node._last_nav_progress_time = 0.0
    node._pending_reset = None
    node._reset_results = OrderedDict()
    node._boot_epoch = 0
    node._position_timeout = 2.0
    node._setorigin_timeout = 3.0
    node._reset_frame_timeout = 1.0
    node._publish_tf_enabled = True
    for name in ('_odom_pub', '_twist_pub', '_health_pub', '_tf_pub', '_reset_result_pub'):
        setattr(node, name, Publisher())
    node._origin_client = Client()
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        to_msg=lambda: __import__('builtin_interfaces.msg', fromlist=['Time']).Time(sec=100)))
    node.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    return node, clock


def odom(generation=1, stamp=1000, initialized=True, valid=True):
    message = ZitOdom()
    message.pose_odom = [3.0, -2.0, 1.5, 0.1, -0.2, math.pi / 2]
    message.twist_body = [0.3, -0.1, 0.2, 0.01, 0.02, 0.03]
    message.origin_initialized = initialized
    message.nav_valid = valid
    message.origin_generation = generation
    message.nav_timestamp_ms = stamp
    return message


def response(generation=2, stamp=1100, success=True):
    result = SetOrigin.Response()
    result.success = success
    result.message = 'set' if success else 'armed'
    result.origin_nav = [100.0, 200.0, 8.0, 0.0, 0.0, 0.4]
    result.origin_generation = generation
    result.nav_timestamp_ms = stamp
    return result


def reset(node, request_id=11):
    request = StateResetRequest(request_id=request_id)
    node._reset_cb(request)
    return node._origin_client.futures[-1]


def test_counter_comparison_handles_rollover():
    assert _at_least_u32(3, 0xfffffffe)
    assert not _at_least_u32(0xfffffffe, 3)
    assert _at_least_u32(4, 4)


def test_adapter_rejects_nonfinite_and_malformed_feedback():
    assert not EstimatorNode._valid_odom(SimpleNamespace(
        pose_odom=[1.0] * 5, twist_body=[0.0] * 6))
    message = odom()
    message.pose_odom = [float('nan')] + [0.0] * 5
    assert not EstimatorNode._valid_odom(message)


def test_typed_odom_is_forwarded_without_second_origin_transform(adapter):
    node, _clock = adapter
    message = odom()
    node._odom_cb(message)
    node._publish_tick()
    pose = node._odom_pub.messages[-1]
    assert (pose.robot_x, pose.robot_y, pose.robot_z) == (3.0, -2.0, 1.5)
    assert pose.robot_yaw == pytest.approx(90.0)
    assert pose.robot_roll == pytest.approx(math.degrees(0.1))
    assert (pose.origin_x, pose.origin_y, pose.origin_z, pose.origin_yaw) == (0.0,) * 4
    assert pose.origin_generation == 1 and pose.nav_timestamp_ms == 1000
    assert pose.origin_initialized and pose.nav_valid
    assert node._twist_pub.messages[-1].twist.twist.linear.x == pytest.approx(0.3)
    assert node._tf_pub.messages[-1].transforms[0].transform.translation.x == 3.0


def test_valid_navigation_before_first_origin_does_not_deadlock_startup(adapter):
    node, _clock = adapter
    node._odom_cb(odom(generation=0, initialized=False))
    node._publish_tick()
    assert node._health_pub.messages[-1].available
    assert node._odom_pub.messages[-1].nav_valid
    assert not node._odom_pub.messages[-1].origin_initialized
    assert not node._tf_pub.messages


def test_duplicate_raw_timestamp_cannot_keep_stale_nav_healthy(adapter):
    node, clock = adapter
    node._odom_cb(odom())
    clock[0] += 2.1
    node._odom_cb(odom())
    node._publish_tick()
    assert not node._odom_pub.messages[-1].nav_valid
    assert not node._health_pub.messages[-1].available
    assert node._twist_pub.messages[-1].twist.twist.linear.x == 0.0


def test_success_waits_for_valid_frame_matching_response_version_and_sample(adapter):
    node, _clock = adapter
    node._odom_cb(odom())
    future = reset(node)
    future.set_result(response())
    assert not node._reset_result_pub.messages
    node._odom_cb(odom(generation=1, stamp=1050))
    node._odom_cb(odom(generation=2, stamp=1090))
    node._odom_cb(odom(generation=2, stamp=1100, valid=False))
    assert not node._reset_result_pub.messages
    node._odom_cb(odom(generation=2, stamp=1110))
    result = node._reset_result_pub.messages[-1]
    assert result.request_id == 11 and result.success and result.origin_generation == 2
    assert node._odom_pub.messages[-1].robot_x == 3.0


def test_matching_frame_arriving_before_service_reply_is_confirmed(adapter):
    node, _clock = adapter
    node._odom_cb(odom())
    future = reset(node)
    node._odom_cb(odom(generation=2, stamp=1110))
    assert not node._reset_result_pub.messages
    future.set_result(response())
    assert node._reset_result_pub.messages[-1].success


def test_failed_reset_preserves_measured_state_and_repeated_id_does_not_recall(adapter):
    node, _clock = adapter
    original = odom()
    node._odom_cb(original)
    future = reset(node)
    future.set_result(response(success=False))
    assert node._odom is original
    assert not node._reset_result_pub.messages[-1].success
    reset_request = StateResetRequest(request_id=11)
    node._reset_cb(reset_request)
    assert len(node._origin_client.futures) == 1


def test_reset_is_single_flight_and_missing_frame_times_out(adapter):
    node, clock = adapter
    node._odom_cb(odom())
    future = reset(node)
    node._reset_cb(StateResetRequest(request_id=12))
    assert node._reset_result_pub.messages[-1].request_id == 12
    assert not node._reset_result_pub.messages[-1].success
    assert len(node._origin_client.futures) == 1
    future.set_result(response())
    clock[0] += 1.1
    node._reset_timeout_tick()
    assert node._reset_result_pub.messages[-1].request_id == 11
    assert not node._reset_result_pub.messages[-1].success


@pytest.mark.parametrize('message', [
    odom(generation=1, stamp=10),
    odom(generation=0, stamp=1100, initialized=False),
    odom(generation=0, stamp=1100),
])
def test_mcu_reboot_invalidates_inflight_reset_and_ignores_late_reply(adapter, message):
    node, _clock = adapter
    node._odom_cb(odom())
    future = reset(node)
    pending = node._pending_reset
    node._odom_cb(message)
    assert node._pending_reset is None
    assert not node._reset_result_pub.messages[-1].success
    late = Future()
    late.set_result(response())
    count = len(node._reset_result_pub.messages)
    node._origin_done(pending, late)
    assert len(node._reset_result_pub.messages) == count
    assert future.cancelled()


@pytest.mark.parametrize('generation', [0, 1])
def test_response_must_advance_to_nonzero_generation(adapter, generation):
    node, _clock = adapter
    node._odom_cb(odom())
    future = reset(node)
    future.set_result(response(generation=generation))
    assert not node._reset_result_pub.messages[-1].success
    assert node._pending_reset is None


def test_matching_frame_requires_fresh_raw_sample_progress(adapter):
    node, clock = adapter
    node._odom_cb(odom())
    future = reset(node)
    node._odom_cb(odom(generation=2, stamp=1100))
    clock[0] += 2.1
    node._odom_cb(odom(generation=2, stamp=1100))
    future.set_result(response())
    assert not node._reset_result_pub.messages
    node._odom_cb(odom(generation=2, stamp=1110))
    assert node._reset_result_pub.messages[-1].success


def test_response_may_skip_an_externally_updated_origin_generation(adapter):
    node, _clock = adapter
    node._odom_cb(odom())
    future = reset(node)
    future.set_result(response(generation=3))
    assert not node._reset_result_pub.messages
    node._odom_cb(odom(generation=3, stamp=1110))
    assert node._reset_result_pub.messages[-1].success
