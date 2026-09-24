"""Simple constant-position filter used as the first estimator backend."""


def exponential(previous, current, alpha=0.5):
    if previous is None:
        return tuple(current)
    alpha = float(alpha)
    return tuple((1.0 - alpha) * float(a) + alpha * float(b)
                 for a, b in zip(previous, current))
