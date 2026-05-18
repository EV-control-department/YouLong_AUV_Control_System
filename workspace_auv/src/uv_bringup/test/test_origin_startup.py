"""Exercise the startup origin handshake without changing physical hardware."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS

import pytest


class Blocked(RuntimeError):
    pass


@pytest.fixture
def manager():
    path = Path(__file__).parents[1] / 'uv_bringup/real_startup.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'RealStartupManager')
    names = {'_origin_navigation_ready', '_origin_initialized',
             '_origin_ready_to_set', '_initialize_origin', '_raw_odom_cb', '_mcu_cb'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    clock = NS(value=10.0)
    ns = {'time': NS(monotonic=lambda: clock.value), 'math': math,
          'StartupBlocked': Blocked, 'ShutdownRequested': type('Shutdown', (Exception,), {}),
          'SetOrigin': NS(Request=lambda: object())}
    shell = ast.ClassDef(name='Manager', bases=[], keywords=[], body=methods, decorator_list=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=[shell], type_ignores=[])), str(path), 'exec'), ns)
    m = ns['Manager']()
    m.clock = clock
    m.max_age, m.timeout = 2.0, 120.0
    m.args = NS(auto_setorigin=True, startup_mode='auto')
    m._mcu_status = NS(ins_state=4, navigation_ready=True, is_armed=False, error_flags=0)
    m._mcu_status_at = 10.0
    m._raw_odom = NS(nav_valid=True, origin_initialized=False, origin_generation=0,
                     nav_timestamp_ms=100, pose_odom=[0.] * 6, twist_body=[0.] * 6)
    m._raw_odom_at = m._raw_progress_at = m._last_odom = 10.0
    m._raw_progress_samples = 3
    m._origin_pose = NS(origin_initialized=False, origin_generation=0)
    m._ins_ready_since = None
    m.component_state = {}
    m.calls = []
    m.response = NS(success=True, origin_generation=1, nav_timestamp_ms=101,
                    origin_nav=[0.] * 6, message='ok')
    def call(request):
        m.calls.append(request)
        return NS(done=lambda: True, result=lambda: m.response)
    m._origin_client = NS(service_is_ready=lambda: True, call_async=call)
    m.get_logger = lambda: NS(info=lambda text: None)
    m._wait_for = lambda predicate, description, **kw: predicate() or pytest.fail(description)
    return m


@pytest.mark.parametrize('field,value', [('ins_state', 0), ('ins_state', 1),
    ('ins_state', 2), ('ins_state', 5), ('navigation_ready', False),
    ('is_armed', True), ('error_flags', 1)])
def test_unready_or_armed_mcu_never_releases_setorigin(manager, field, value):
    setattr(manager._mcu_status, field, value)
    assert not manager._origin_ready_to_set()
    assert not manager.calls


@pytest.mark.parametrize('field,value', [('_mcu_status_at', 7.),
    ('_raw_odom_at', 7.), ('_raw_progress_at', 7.), ('_raw_progress_samples', 1)])
def test_stale_or_nonadvancing_data_never_releases_setorigin(manager, field, value):
    setattr(manager, field, value)
    assert not manager._origin_ready_to_set()


def test_ready_must_remain_stable_and_service_available(manager):
    assert not manager._origin_ready_to_set()
    manager.clock.value += .5
    assert not manager._origin_ready_to_set()
    manager._mcu_cb(NS(ins_state=2, navigation_ready=False, is_armed=False, error_flags=0))
    manager._mcu_cb(NS(ins_state=3, navigation_ready=True, is_armed=False, error_flags=0))
    assert not manager._origin_ready_to_set()
    manager.clock.value += 1.0
    manager._origin_client.service_is_ready = lambda: False
    assert not manager._origin_ready_to_set()
    manager._origin_client.service_is_ready = lambda: True
    assert manager._origin_ready_to_set()


def test_repeated_timestamp_does_not_extend_navigation_freshness(manager):
    manager.clock.value = 13.
    manager._raw_odom_cb(manager._raw_odom)
    assert manager._raw_odom_at == 13.
    assert manager._raw_progress_at == 10.
    assert not manager._origin_navigation_ready()


def test_initialized_origin_is_preserved_even_when_armed(manager):
    manager._raw_odom.origin_initialized = manager._origin_pose.origin_initialized = True
    manager._raw_odom.origin_generation = manager._origin_pose.origin_generation = 6
    manager._mcu_status.is_armed = True
    manager._phase = lambda name, predicate, description: predicate() or pytest.fail(description)
    manager._initialize_origin()
    assert not manager.calls
    assert manager.component_state['origin'].startswith('REUSED')


@pytest.mark.parametrize('mode,enabled', [('adopt', True), ('auto', False)])
def test_observation_or_disabled_mode_never_calls_service(manager, mode, enabled):
    manager.args.startup_mode, manager.args.auto_setorigin = mode, enabled
    manager._initialize_origin()
    assert not manager.calls


def test_success_waits_for_matching_mcu_and_localization_generation(manager):
    phases = []
    def phase(name, predicate, description):
        phases.append(description)
        if len(phases) == 1:
            assert not predicate()
            manager.clock.value += 1.0
            assert predicate()
        else:
            assert not predicate()
            manager._raw_odom.origin_initialized = True
            manager._raw_odom.origin_generation = 1
            manager._raw_odom.nav_timestamp_ms = 101
            assert not predicate()  # localization has not adopted it
            manager._origin_pose.origin_initialized = True
            manager._origin_pose.origin_generation = 2
            assert not predicate()  # wrong version
            manager._origin_pose.origin_generation = 1
            assert predicate()
    manager._phase = phase
    manager._initialize_origin()
    assert len(manager.calls) == 1
    assert len(phases) == 2


@pytest.mark.parametrize('success,generation', [(False, 1), (True, 0), (True, 0xffffffff)])
def test_rejected_or_invalid_response_blocks_without_retry(manager, success, generation):
    manager._phase = lambda *args: None
    manager.response.success, manager.response.origin_generation = success, generation
    with pytest.raises(Blocked):
        manager._initialize_origin()
    assert len(manager.calls) == 1


def test_existing_raw_origin_is_not_reset_while_pose_catches_up(manager):
    manager._raw_odom.origin_initialized = True
    manager._raw_odom.origin_generation = 4
    phases = []
    def phase(name, predicate, description):
        phases.append(description)
        if len(phases) == 2:
            manager._origin_pose.origin_initialized = True
            manager._origin_pose.origin_generation = 4
            assert predicate()
    manager._phase = phase
    manager._initialize_origin()
    assert not manager.calls
    assert len(phases) == 2


def test_service_timeout_cancels_pending_request_without_retry(manager):
    manager._phase = lambda *args: None
    cancelled, removed = [], []
    future = NS(done=lambda: False, cancel=lambda: cancelled.append(True))
    manager._origin_client.call_async = lambda request: manager.calls.append(request) or future
    manager._origin_client.remove_pending_request = lambda pending: removed.append(pending)
    def timeout(*args, **kwargs):
        raise Blocked('timeout')
    manager._wait_for = timeout
    with pytest.raises(Blocked, match='did not complete'):
        manager._initialize_origin()
    assert len(manager.calls) == 1
    assert cancelled == [True] and removed == [future]


def test_existing_origin_can_propagate_while_vehicle_is_armed(manager):
    manager._raw_odom.origin_initialized = True
    manager._mcu_status.is_armed = True
    assert manager._origin_ready_to_set()


def test_confirmation_rejects_stale_odom_and_earlier_timestamp(manager):
    manager._raw_odom.origin_initialized = manager._origin_pose.origin_initialized = True
    manager._raw_odom.origin_generation = manager._origin_pose.origin_generation = 1
    assert not manager._origin_initialized(1, 101)
    manager._raw_odom.nav_timestamp_ms = 101
    assert manager._origin_initialized(1, 101)
    manager._raw_progress_at = 7.0
    assert not manager._origin_initialized(1, 101)
