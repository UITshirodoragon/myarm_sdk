"""In-memory robot-arm adapter for tests and demos."""

from __future__ import annotations

from typing import Optional

from myarm_sdk.core import JointPositions


class FakeRobotArmAdapter:
    """Store joint state without connecting to physical hardware."""

    def __init__(self, initial: Optional[JointPositions] = None) -> None:
        self._joints = initial or JointPositions((0.0,) * 6)

    def read_joints(self) -> JointPositions:
        return self._joints

    def move_joints(self, target: JointPositions, speed: int = 50) -> None:
        if not 1 <= speed <= 100:
            raise ValueError("speed must be in the range 1..100")
        self._joints = target

    def close(self) -> None:
        pass
