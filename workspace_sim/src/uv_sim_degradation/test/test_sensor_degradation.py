import random

import numpy as np
from sensor_msgs.msg import Image, Imu
from uv_msgs.msg import UsblMeasurement

from uv_sim_degradation.camera_degradation import degrade_image, keep_image
from uv_sim_degradation.imu_degradation import degrade_imu
from uv_sim_degradation.usbl_degradation import degrade_usbl


def test_camera_dropout_is_reproducible_and_preserves_message():
    message = Image()
    assert keep_image(message, rng=random.Random(4), dropout_probability=0.0) is message
    assert keep_image(message, rng=random.Random(1), dropout_probability=1.0) is None


def test_camera_brightness_degradation_changes_pixels_without_mutating_input():
    message = Image()
    message.width = 2
    message.height = 1
    message.encoding = 'mono8'
    message.step = 2
    message.data = np.array([100, 200], dtype=np.uint8).tobytes()
    output = degrade_image(
        message, rng=random.Random(4), brightness_scale=0.5)
    assert output is not message
    assert list(output.data) == [50, 100]
    assert list(message.data) == [100, 200]


def test_imu_bias_is_applied_without_mutating_input():
    message = Imu()
    message.linear_acceleration.x = 1.0
    output = degrade_imu(
        message, rng=random.Random(4), accelerometer_bias=(0.5, 0.0, 0.0))
    assert output is not None
    assert output.linear_acceleration.x == 1.5
    assert message.linear_acceleration.x == 1.0


def test_usbl_noise_zero_keeps_measurement_valid():
    message = UsblMeasurement()
    message.position.x = 2.0
    message.valid = True
    output = degrade_usbl(
        message, rng=random.Random(4), position_noise_stddev=0.0)
    assert output is not None
    assert output.position.x == 2.0
    assert output.valid


def test_usbl_outlier_keeps_sample_valid_but_changes_position():
    message = UsblMeasurement()
    message.position.x = 2.0
    message.valid = True
    output = degrade_usbl(
        message, rng=random.Random(4), outlier_probability=1.0,
        outlier_stddev=10.0)
    assert output is not None
    assert output.valid
    assert output.position.x != 2.0
