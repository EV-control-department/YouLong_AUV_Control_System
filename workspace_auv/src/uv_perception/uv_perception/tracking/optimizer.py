"""Optimization backend seam for later factor-graph integration."""


class Optimizer:
    """Keep the estimator independent from a particular solver library."""

    def update(self, states, observations):
        del observations
        return states
