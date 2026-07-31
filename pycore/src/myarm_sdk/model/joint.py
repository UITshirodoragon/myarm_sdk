"""Joint-space value objects."""

from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class JointPositions:
    """Joint positions in radians, ordered from joint 1 through joint 6."""

    values: Tuple[float, ...]

    def __init__(self, values: Sequence[float]) -> None:
        normalized = tuple(float(value) for value in values)
        if len(normalized) != 6:
            raise ValueError("MyArm M750 requires exactly 6 joint positions")
        object.__setattr__(self, "values", normalized)
