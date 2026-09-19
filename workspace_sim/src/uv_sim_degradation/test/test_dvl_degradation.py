import random

from uv_msgs.msg import DvlAltitude
from uv_sim_degradation.dvl_degradation import (
    degrade_altitude, degrade_velocity,
)


def test_dropout_is_reproducible_with_a_seed():
    kwargs = {'rng': random.Random(42), 'dropout_probability': 0.5,
              'noise_stddev': 0.0}
    first = [degrade_velocity((1.0, 2.0, 3.0), **kwargs) for _ in range(8)]
    kwargs = {'rng': random.Random(42), 'dropout_probability': 0.5,
              'noise_stddev': 0.0}
    second = [degrade_velocity((1.0, 2.0, 3.0), **kwargs) for _ in range(8)]
    assert first == second


def test_zero_noise_preserves_valid_velocity():
    result = degrade_velocity((1.0, -2.0, 0.5), rng=random.Random(1),
                              dropout_probability=0.0, noise_stddev=0.0)
    assert result == (1.0, -2.0, 0.5)


def test_bottom_lock_loss_marks_altitude_invalid():
    message = DvlAltitude()
    message.altitude = 1.5
    message.valid = True
    result = degrade_altitude(
        message, rng=random.Random(1), bottom_lock_loss_probability=1.0)
    assert result is not None
    assert result.altitude == 1.5
    assert not result.valid
