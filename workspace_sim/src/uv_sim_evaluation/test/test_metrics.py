from uv_sim_evaluation.metrics import ate_rmse, rpe_rotation, rpe_translation


def test_ate_and_rpe_are_zero_for_identical_tracks():
    track = [(0.0, 0.0, 0.0), (1.0, 2.0, 0.5)]
    assert ate_rmse(track, track) == 0.0
    assert rpe_translation(track, track) == 0.0


def test_metrics_report_a_two_meter_constant_bias():
    truth = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    estimate = [(2.0, 0.0, 0.0), (3.0, 0.0, 0.0)]
    assert abs(ate_rmse(estimate, truth) - 2.0) < 1e-9
    assert rpe_translation(estimate, truth) == 0.0


def test_rpe_rotation_wraps_yaw():
    assert rpe_rotation([(0, 0, 0, 3.13), (0, 0, 0, -3.13)],
                        [(0, 0, 0, 3.13), (0, 0, 0, 3.13)]) < 0.03
