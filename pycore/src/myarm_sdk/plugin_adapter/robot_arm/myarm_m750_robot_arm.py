"""Optional pymycobot adapter for a physical MyArm M750."""

import math
from typing import Any

from myarm_sdk.core import JointPositions


class MyArmM750RobotArmAdapter:
    """Translate canonical radians to the pymycobot hardware API."""

    def __init__(self, serial_port: str, baudrate: int = 115200) -> None:
        try:
            from pymycobot.myarmm import MyArmM
        except ImportError as error:
            raise RuntimeError(
                "Install robot support with `pip install myarm-sdk[robot-arm]`."
            ) from error
        self._arm: Any = MyArmM(serial_port, baudrate)

    def read_joints(self) -> JointPositions:
        angles = self._arm.get_angles()
        if angles is None or len(angles) != 6:
            raise RuntimeError("pymycobot did not return six joint angles")
        return JointPositions(tuple(math.radians(angle) for angle in angles))

    def move_joints(self, target: JointPositions, speed: int = 50) -> None:
        if not 1 <= speed <= 100:
            raise ValueError("speed must be in the range 1..100")
        self._arm.send_angles([math.degrees(value) for value in target.values], speed)

    def close(self) -> None:
        close = getattr(self._arm, "close", None)
        if close is not None:
            close()
