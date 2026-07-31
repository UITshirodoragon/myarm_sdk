"""General data models used at adapter boundaries."""

from dataclasses import dataclass
from typing import Any, Optional, Tuple

from .joint import JointPositions


@dataclass(frozen=True)
class Pose:
    """Cartesian position in metres and quaternion `(x, y, z, w)`."""

    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]


@dataclass(frozen=True)
class CameraFrame:
    """A frame payload without imposing an image-library dependency."""

    data: Any
    timestamp_s: float
    encoding: str = "bgr8"


@dataclass(frozen=True)
class TrajectoryPoint:
    """A joint waypoint scheduled at `time_from_start_s`."""

    positions: JointPositions
    time_from_start_s: float
    velocities: Optional[JointPositions] = None
