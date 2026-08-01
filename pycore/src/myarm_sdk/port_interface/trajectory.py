"""Joint trajectory-planning contract."""

from typing import Protocol

from myarm_sdk.core import TrajectoryPlanningRequest, TrajectoryPlanningResult


class TrajectoryPlannerInterface(Protocol):
    """Plan one validated joint-space trajectory without robot I/O."""

    def plan(self, request: TrajectoryPlanningRequest) -> TrajectoryPlanningResult:
        """Return a safe result instead of an invalid command trajectory."""
        ...
