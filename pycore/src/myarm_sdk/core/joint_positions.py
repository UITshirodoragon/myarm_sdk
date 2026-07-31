"""Joint-space value types for the six-axis MyArm M750."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class JointPositions:
    """Six MyArm M750 joint positions in radians and canonical URDF order."""

    values: Tuple[float, ...]

    def __init__(self, values: Sequence[float]) -> None:
        normalized = tuple(float(value) for value in values)
        if len(normalized) != 6:
            raise ValueError("MyArm M750 requires exactly 6 joint positions")
        if not all(math.isfinite(value) for value in normalized):
            raise ValueError("joint positions must be finite")
        object.__setattr__(self, "values", normalized)
