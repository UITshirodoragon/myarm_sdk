"""ROS-independent intrinsic camera calibration value type."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class CameraCalibration:
    """Intrinsic calibration for one camera instance and raw image mode."""

    calibration_id: str
    source_sha256: str
    width: int
    height: int
    distortion_model: str
    k: Tuple[float, ...]
    d: Tuple[float, ...]
