"""Canonical joint metadata read from the selected robot URDF."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class JointMetadata:
    """One actuated URDF joint expressed in the canonical model convention."""

    name: str
    axis_xyz: Tuple[float, float, float]
    lower_limit_rad: float
    upper_limit_rad: float
    velocity_limit_rad_s: float

    @property
    def positive_direction(self) -> str:
        """Describe URDF's standard positive revolute-joint convention."""
        return "right_hand_rule_about_urdf_axis"

