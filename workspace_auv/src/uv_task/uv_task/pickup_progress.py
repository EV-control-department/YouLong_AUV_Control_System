"""Immutable pickup progress shared by tasks in one runner (raw odom/NED)."""
from dataclasses import dataclass
from typing import Optional, Tuple


@dataclass(frozen=True)
class PickupProgress:
    start_xy: Tuple[float, float] = (0.0, 0.0)
    frame_pose: Optional[Tuple[float, float, float, float]] = None
    depth: Optional[float] = None
    golf_camera_pose: Optional[Tuple[float, float, float, float]] = None
    ring_camera_pose: Optional[Tuple[float, float, float, float]] = None
    golf_attempts: int = 0
    ring_attempts: int = 0
    golf_status: str = 'pending'
    ring_status: str = 'pending'
