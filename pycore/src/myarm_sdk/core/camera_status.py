"""Camera lifecycle and health value type."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class CameraStatus:
    """Snapshot of a camera service without ROS dependencies."""

    instance_id: str
    state: str
    requested_width: int
    requested_height: int
    requested_fps: float
    requested_pixel_format: str
    actual_width: Optional[int]
    actual_height: Optional[int]
    actual_fps: Optional[float]
    actual_pixel_format: Optional[str]
    frame_count: int
    capture_error_count: int
    last_frame_timestamp_s: Optional[float]
    last_error: Optional[str]
    retry_after_monotonic_s: Optional[float]
