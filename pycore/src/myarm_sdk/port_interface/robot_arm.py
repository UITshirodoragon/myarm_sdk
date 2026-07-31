"""Hardware-facing robot-arm contract."""

from typing import Protocol

from myarm_sdk.core import JointPositions


class RobotArmInterface(Protocol):
    """Read and command a physical or simulated robot arm."""

    def read_joints(self) -> JointPositions:
        ...

    def move_joints(self, target: JointPositions, speed: int = 50) -> None:
        ...

    def close(self) -> None:
        ...
