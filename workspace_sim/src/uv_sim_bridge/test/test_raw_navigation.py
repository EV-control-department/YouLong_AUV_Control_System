import math
import pytest

from uv_sim_bridge.raw_navigation import RawNavigation, remove_sensor_lever_arm
from uv_sim_bridge.arm_lifecycle import ArmLifecycle


def test_raw_navigation_has_no_reset_or_odom_feedback_dependency():
    navigation = RawNavigation()
    navigation.update_velocity((1, 0, 0), 0)
    navigation.update_imu((0, 0, 0), 0)
    first = navigation.snapshot(0)
    assert first.valid
    assert navigation.snapshot(0.1).position[0] == pytest.approx(0.1)
    assert navigation.snapshot(0.2).position[0] == pytest.approx(0.2)
    assert not hasattr(navigation, 'reset')


def test_stale_inputs_stop_integration_and_mark_nav_invalid():
    navigation = RawNavigation(timeout_s=0.2)
    navigation.update_velocity((1, 0, 0), 0)
    navigation.update_imu((0, 0, 0), 0)
    navigation.snapshot(0)
    valid = navigation.snapshot(0.1)
    stale = navigation.snapshot(0.4)
    assert not stale.valid
    assert stale.position == valid.position
    assert stale.velocity == (0,) * 6
    assert stale.timestamp_ms == valid.timestamp_ms


def test_turning_dvl_sensor_lever_arm_does_not_invent_translation():
    assert remove_sensor_lever_arm((0, -0.375, 0), (0, 0, 1)) == pytest.approx((0, 0, 0))
    navigation = RawNavigation()
    navigation.update_velocity((0, -0.375, 0), 0)
    navigation.update_imu((0, 0, 1), 0)
    navigation.snapshot(0)
    result = navigation.snapshot(0.1)
    assert result.position[:3] == pytest.approx((0, 0, 0))
    assert result.position[5] == pytest.approx(0.1)


def arm_sequence(lifecycle, *, mode=1, origin_ready=True, nav_ready=True,
                 start=0.0):
    for tick in range(16):
        lifecycle.heartbeat(mode, start + tick / 15,
                            origin_ready=origin_ready, nav_ready=nav_ready)


def test_initially_disarmed_and_arm_requires_explicit_origin_in_all_modes():
    for mode in (1, 3):
        lifecycle = ArmLifecycle()
        assert not lifecycle.armed
        arm_sequence(lifecycle, mode=mode, origin_ready=False)
        assert not lifecycle.armed
    lifecycle = ArmLifecycle()
    arm_sequence(lifecycle, mode=1, nav_ready=False)
    assert not lifecycle.armed
    arm_sequence(lifecycle, mode=3, nav_ready=False, start=1.1)
    assert lifecycle.armed


def test_arm_requires_ten_heartbeats_and_one_second_of_continuity():
    lifecycle = ArmLifecycle()
    for tick in range(10):
        lifecycle.heartbeat(1, tick / 15, origin_ready=True, nav_ready=True)
    assert lifecycle.heartbeat_count == 10
    assert not lifecycle.armed
    lifecycle.check(1.0, origin_ready=True, nav_ready=True)
    assert lifecycle.armed
    lifecycle = ArmLifecycle()
    for tick in range(9):
        lifecycle.heartbeat(1, tick / 8, origin_ready=True, nav_ready=True)
    assert not lifecycle.armed
    lifecycle.heartbeat(1, 1.1, origin_ready=True, nav_ready=True)
    assert lifecycle.armed


def test_timeout_disarms_and_zero_heartbeat_does_not_disarm():
    lifecycle = ArmLifecycle()
    arm_sequence(lifecycle)
    lifecycle.heartbeat(0, 1.1, origin_ready=True, nav_ready=True)
    assert lifecycle.armed
    assert not lifecycle.check_timeout(2.09)
    assert lifecycle.check_timeout(2.11)
    assert not lifecycle.armed
    assert lifecycle.heartbeat_count == 0
    lifecycle.heartbeat(1, 2.2, origin_ready=True, nav_ready=True)
    assert lifecycle.heartbeat_count == 1
    assert not lifecycle.armed


def test_a_stale_unarmed_sequence_cannot_rearm_after_a_gap():
    lifecycle = ArmLifecycle()
    for tick in range(12):
        lifecycle.heartbeat(1, tick / 15, origin_ready=True, nav_ready=True)
    assert not lifecycle.armed
    lifecycle.heartbeat(1, 3.0, origin_ready=True, nav_ready=True)
    assert lifecycle.heartbeat_count == 1
    assert not lifecycle.armed


def test_sil_nav_timestamp_uses_the_backend_boot_clock():
    navigation = RawNavigation(timestamp_origin_s=100.0)
    navigation.update_velocity((0, 0, 0), 100.0)
    navigation.update_imu((0, 0, 0), 100.0)
    assert navigation.snapshot(100.0).timestamp_ms == 0
    assert navigation.snapshot(100.1).timestamp_ms in (99, 100)
