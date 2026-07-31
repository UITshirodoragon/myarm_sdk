from typing import Protocol

from myarm_sdk.model import JointPositions


class JointPositionController(Protocol):
    """Send and inspect joint-space position setpoints."""

    def command(self, target: JointPositions) -> None:
        ...

    def last_command(self) -> JointPositions:
        ...
