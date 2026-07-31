"""In-memory controller placeholder useful in tests and demos."""

from __future__ import annotations

from typing import Optional

from myarm_sdk.core import JointPositions


class MemoryControllerAdapter:
    """Retain the most recent joint target without hardware I/O."""

    def __init__(self, initial: Optional[JointPositions] = None) -> None:
        self._last_command = initial or JointPositions((0.0,) * 6)

    def command(self, target: JointPositions) -> None:
        self._last_command = target

    def last_command(self) -> JointPositions:
        return self._last_command
