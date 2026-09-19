"""Perception boundary for detections, features and target observations.

The current detector implementation remains in :mod:`uv_camera` while the
public topic contract lives here.  Moving the model runtime behind this
boundary later will not change consumers such as ``uv_task`` or localization.
"""
