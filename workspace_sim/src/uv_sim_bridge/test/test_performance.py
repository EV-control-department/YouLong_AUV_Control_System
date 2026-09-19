from uv_sim_bridge.performance import ControlLoopStats


def test_control_stats_reports_frequency_and_jitter():
    stats = ControlLoopStats(target_hz=100.0)
    for timestamp in (0.00, 0.01, 0.02, 0.03):
        stats.record_tick(timestamp)

    result = stats.snapshot(0.04)
    assert result['ticks'] == 4
    assert result['period_samples'] == 3
    assert abs(result['frequency_hz'] - 100.0) < 1e-9
    assert result['period_mean_ms'] == 10.0
    assert result['max_jitter_ms'] < 1e-9
    assert result['deadline_misses'] == 0


def test_control_stats_marks_late_deadlines_and_can_reset():
    stats = ControlLoopStats(target_hz=100.0)
    stats.record_tick(0.00)
    stats.record_tick(0.02, late_by_s=0.003)
    result = stats.snapshot(0.03)
    assert result['deadline_misses'] == 1
    assert result['max_jitter_ms'] == 10.0

    reset_result = stats.snapshot(0.04, reset=True)
    assert reset_result['ticks'] == 2
    assert stats.snapshot(0.05)['ticks'] == 0
