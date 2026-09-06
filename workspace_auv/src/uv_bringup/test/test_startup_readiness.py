"""A single message or stale pre-start sample must not release the task."""

from uv_bringup.wait_for_sim import Readiness


def test_requires_two_samples_from_every_dependency():
    state = Readiness({'odom', 'front'})
    state.observe('odom', 1.0)
    state.observe('odom', 1.1)
    state.observe('front', 1.1)
    assert state.missing(1.2) == ['front']
    state.observe('front', 1.2)
    assert state.missing(1.3) == []


def test_stale_sensor_cannot_release_gate_with_new_camera_data():
    state = Readiness({'odom', 'front'})
    for t in (1.0, 1.1):
        state.observe('odom', t)
    for t in (4.0, 4.1):
        state.observe('front', t)
    assert state.missing(4.1) == ['odom']
    state.observe('odom', 4.2)
    assert state.missing(4.2) == ['odom']
    state.observe('odom', 4.3)
    assert state.missing(4.3) == []
