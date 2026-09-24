"""Conservative geometry covariance defaults."""


def diagonal(value=1.0):
    value = float(value)
    return [value, 0.0, 0.0, 0.0, value, 0.0, 0.0, 0.0, value]
