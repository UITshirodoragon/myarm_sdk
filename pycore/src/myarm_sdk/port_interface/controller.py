"""Joint command controller contract."""

from typing import Protocol

from myarm_sdk.core import JointPositions


class ControllerInterface(Protocol):
    """Send and inspect joint-space position targets."""

    def command(self, target: JointPositions) -> None:
        ...

    def last_command(self) -> JointPositions:
        ...
