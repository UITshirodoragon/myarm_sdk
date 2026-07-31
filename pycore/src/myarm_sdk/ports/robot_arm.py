from typing import Protocol

from myarm_sdk.model import JointPositions


class RobotArm(Protocol):
    """Minimum hardware-facing robot-arm contract."""

    def read_joints(self) -> JointPositions:
        ...

    def move_joints(self, target: JointPositions, speed: int = 50) -> None:
        ...

    def close(self) -> None:
        ...
