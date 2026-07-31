"""Timed joint trajectory value type."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .joint_positions import JointPositions


@dataclass(frozen=True)
class TrajectoryPoint:
    """A joint waypoint scheduled at ``time_from_start_s``."""

    positions: JointPositions
    time_from_start_s: float
    velocities: Optional[JointPositions] = None
