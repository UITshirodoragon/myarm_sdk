"""Cartesian pose value type."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class Pose:
    """Cartesian position in metres and quaternion in ``(x, y, z, w)`` order."""

    position: Tuple[float, float, float]
    orientation: Tuple[float, float, float, float]
