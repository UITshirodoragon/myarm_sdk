"""In-memory controller adapter useful in integration tests."""

from myarm_sdk.model import JointPositions


class MemoryJointPositionController:
    def __init__(self, initial: JointPositions = None) -> None:
        self._last_command = initial or JointPositions((0, 0, 0, 0, 0, 0))

    def command(self, target: JointPositions) -> None:
        self._last_command = target

    def last_command(self) -> JointPositions:
        return self._last_command
