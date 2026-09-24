"""Canonical perception topic helpers.

This module is the stable semantic interface for the perception graph. Image
payloads are intentionally not represented here: raw pixels use iceoryx2
services and only metadata/semantic results use ROS 2.
"""

from auv_protocol.topics import (
    ARUCO_IDS,
    DETECTIONS,
    LINES,
    MEASUREMENTS,
    OBJECTS,
    PERCEPTION_DETECTIONS,
    TARGET_OBSERVATIONS,
    TARGETS,
    TRACKS,
)

__all__ = [
    'ARUCO_IDS', 'DETECTIONS', 'LINES', 'MEASUREMENTS', 'OBJECTS',
    'PERCEPTION_DETECTIONS', 'TARGETS', 'TARGET_OBSERVATIONS', 'TRACKS',
]
