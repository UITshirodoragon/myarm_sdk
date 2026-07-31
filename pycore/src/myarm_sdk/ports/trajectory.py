from typing import Protocol, Sequence

from myarm_sdk.model import JointPositions, TrajectoryPoint


class TrajectoryPlanner(Protocol):
    """Plan timed joint-space waypoints."""

    def plan(
        self, start: JointPositions, target: JointPositions, duration_s: float
    ) -> Sequence[TrajectoryPoint]:
        ...
