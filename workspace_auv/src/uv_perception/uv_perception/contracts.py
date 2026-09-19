"""Canonical perception topic helpers.

This module deliberately has no ROS node implementation.  It is the stable
import surface for perception publishers and consumers while the existing
``uv_camera`` process is migrated incrementally.
"""

from auv_protocol.topics import (
    ARUCO_IDS,
    DETECTIONS,
    LINES,
    OBJECTS,
    TARGET_OBSERVATIONS,
    TARGETS,
)

__all__ = [
    'ARUCO_IDS', 'DETECTIONS', 'LINES', 'OBJECTS', 'TARGETS',
    'TARGET_OBSERVATIONS',
]
