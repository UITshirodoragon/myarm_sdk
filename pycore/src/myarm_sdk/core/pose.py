"""Cartesian pose value type."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class Pose:
    """Cartesian position in metres and quaternion in ``(x, y, z, w)`` order."""

    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]

    def __init__(
        self,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> None:
        normalized_position = tuple(float(value) for value in position)
        normalized_orientation = tuple(float(value) for value in orientation)
        if len(normalized_position) != 3 or len(normalized_orientation) != 4:
            raise ValueError("Pose requires a 3D position and an xyzw quaternion")
        if not all(math.isfinite(value) for value in normalized_position + normalized_orientation):
            raise ValueError("Pose values must be finite")
        quaternion_norm = math.sqrt(
            sum(value * value for value in normalized_orientation)
        )
        if quaternion_norm < 1e-12:
            raise ValueError("Pose orientation quaternion must not have zero length")
        object.__setattr__(self, "position", normalized_position)
        object.__setattr__(
            self,
            "orientation",
            tuple(value / quaternion_norm for value in normalized_orientation),
        )
