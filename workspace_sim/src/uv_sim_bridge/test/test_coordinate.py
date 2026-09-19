from uv_sim_bridge.coordinate import scene_to_odom_ned, scene_yaw_to_odom_ned
from uv_sim_bridge.sensor_adapter import dvl_to_body_velocity


def test_scene_to_odom_uses_canonical_ned_convention():
    assert scene_to_odom_ned(2.0, 3.0, 4.0) == (3.0, -2.0, 4.0)
    assert abs(scene_yaw_to_odom_ned(1.5707963267948966)) < 1e-9


def test_sim_dvl_mount_rotation_maps_sensor_axes_to_body_frd():
    # The simulated DVL's rpy=(pi, 0, -45deg) maps a forward body velocity
    # to equal and opposite x/y sensor components.
    vx, vy, vz = dvl_to_body_velocity(1.0, -1.0, 0.0)
    assert vx > 1.4
    assert abs(vy) < 1e-3
    assert abs(vz) < 1e-5
