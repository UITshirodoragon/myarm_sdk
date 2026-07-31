"""Controller capability placeholder."""

from myarm_sdk.core import JointPositions
from myarm_sdk.port_interface import ControllerInterface


class ControllerService:
    """Thin placeholder that owns a controller interface for future hardware use."""

    def __init__(self, controller: ControllerInterface) -> None:
        self._controller = controller

    def command(self, target: JointPositions) -> None:
        self._controller.command(target)

    def last_command(self) -> JointPositions:
        return self._controller.last_command()
