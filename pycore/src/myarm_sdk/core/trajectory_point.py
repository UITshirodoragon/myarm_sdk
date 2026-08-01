"""Timed joint trajectory point value type."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from .joint_positions import JointPositions


@dataclass(frozen=True)
class TrajectoryPoint:
    """A joint waypoint scheduled at ``time_from_start_s``.

    Velocity and acceleration are optional so this type can also represent an
    externally supplied trajectory.  The minimum-jerk planner always fills
    both fields, and :class:`JointTrajectory` validates timing independently
    of the trajectory source.
    """

    positions: JointPositions
    time_from_start_s: float
    velocities: Optional[JointPositions] = None
    accelerations: Optional[JointPositions] = None

    def __post_init__(self) -> None:
        time_from_start_s = float(self.time_from_start_s)
        if not math.isfinite(time_from_start_s) or time_from_start_s < 0.0:
            raise ValueError("time_from_start_s must be finite and non-negative")
        object.__setattr__(self, "time_from_start_s", time_from_start_s)
