"""Optional pymycobot adapter for a physical MyArm M750."""

from __future__ import annotations

import math
from typing import Any, Sequence, Tuple

from myarm_sdk.core import JointPositions


class MyArmM750RobotArmAdapter:
    """Translate canonical URDF radians to the pymycobot hardware convention.

    The active PoE URDF deliberately keeps q2/q3 firmware calibration out of
    joint origins.  This adapter owns that hardware-to-model conversion so
    feedback can be used safely as a Pinocchio IK seed.
    """

    DEFAULT_MODEL_TO_HARDWARE_OFFSETS_RAD = (
        0.0,
        math.radians(10.0),
        math.radians(-10.0),
        0.0,
        0.0,
        0.0,
    )

    def __init__(
        self,
        serial_port: str,
        baudrate: int = 115200,
        model_to_hardware_offsets_rad: Sequence[float] = (
            DEFAULT_MODEL_TO_HARDWARE_OFFSETS_RAD
        ),
    ) -> None:
        try:
            from pymycobot.myarmm import MyArmM
        except ImportError as error:
            raise RuntimeError(
                "Install robot support with `pip install myarm-sdk[robot-arm]`."
            ) from error
        self._arm: Any = MyArmM(serial_port, baudrate)
        offsets = tuple(float(value) for value in model_to_hardware_offsets_rad)
        if len(offsets) != 6 or not all(math.isfinite(value) for value in offsets):
            raise ValueError("model_to_hardware_offsets_rad requires six finite values")
        self._model_to_hardware_offsets_rad = offsets

    def read_joints(self) -> JointPositions:
        angles = self._arm.get_angles()
        if angles is None or len(angles) != 6:
            raise RuntimeError("pymycobot did not return six joint angles")
        hardware_positions = tuple(math.radians(angle) for angle in angles)
        return self.model_from_hardware_positions(hardware_positions)

    def move_joints(self, target: JointPositions, speed: int = 50) -> None:
        if not 1 <= speed <= 100:
            raise ValueError("speed must be in the range 1..100")
        hardware_positions = self.hardware_from_model_positions(target)
        self._arm.send_angles([math.degrees(value) for value in hardware_positions], speed)

    def close(self) -> None:
        close = getattr(self._arm, "close", None)
        if close is not None:
            close()

    def model_from_hardware_positions(
        self, hardware_positions_rad: Sequence[float]
    ) -> JointPositions:
        """Convert feedback q_real into canonical URDF/Pinocchio q_model."""
        values = tuple(float(value) for value in hardware_positions_rad)
        if len(values) != 6:
            raise ValueError("hardware feedback must contain six joint values")
        return JointPositions(
            tuple(
                value - offset
                for value, offset in zip(values, self._model_to_hardware_offsets_rad)
            )
        )

    def hardware_from_model_positions(
        self, model_positions: JointPositions
    ) -> Tuple[float, ...]:
        """Convert canonical URDF/Pinocchio q_model into hardware q_real."""
        return tuple(
            value + offset
            for value, offset in zip(
                model_positions.values, self._model_to_hardware_offsets_rad
            )
        )
