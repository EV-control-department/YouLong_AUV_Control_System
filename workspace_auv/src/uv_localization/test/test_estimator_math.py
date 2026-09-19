from uv_localization.estimator import (
    _body_to_world, _remove_sensor_lever_arm, _wrap_rad, EstimatorNode,
)


def test_body_to_world_matches_ned_yaw_convention():
    x, y = _body_to_world(1.0, 0.0, 1.5707963267948966)
    assert abs(x) < 1e-9
    assert abs(y - 1.0) < 1e-9


def test_wrap_rad():
    assert abs(_wrap_rad(4.0) - (4.0 - 2.0 * 3.141592653589793)) < 1e-9


def test_remove_sensor_lever_arm():
    # omega_z=0.2 at r_x=-0.375 contributes -0.075 m/s in sensor y.
    corrected = _remove_sensor_lever_arm(
        (1.0, -0.075, 0.0), (0.0, 0.0, 0.2), (-0.375, 0.0, 0.2),
    )
    assert abs(corrected[0] - 1.0) < 1e-9
    assert abs(corrected[1]) < 1e-9
    assert abs(corrected[2]) < 1e-9


def test_parser_rejects_short_and_nonfinite_samples():
    assert EstimatorNode._parse_pose([1.0, 2.0, 3.0]) is None
    assert EstimatorNode._parse_pose([1.0, 2.0, 3.0, float('nan')]) is None
    assert EstimatorNode._parse_velocity([1.0, 2.0, 3.0]) is None
