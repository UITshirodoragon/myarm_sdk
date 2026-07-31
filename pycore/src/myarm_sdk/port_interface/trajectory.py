"""Joint trajectory planning contract."""

from typing import Protocol, Sequence

from myarm_sdk.core import JointPositions, TrajectoryPoint


class TrajectoryInterface(Protocol):
    """Plan timed joint-space waypoints."""

    def plan(
        self, start: JointPositions, target: JointPositions, duration_s: float
    ) -> Sequence[TrajectoryPoint]:
        ...
