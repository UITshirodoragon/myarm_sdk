"""Joint trajectory-planning contract."""

from typing import Protocol

from myarm_sdk.core import (
    JointTrajectoryPlanningRequest,
    JointTrajectoryPlanningResult,
)


class JointTrajectoryPlannerInterface(Protocol):
    """Plan one validated joint-space trajectory without robot I/O."""

    def plan(
        self, request: JointTrajectoryPlanningRequest
    ) -> JointTrajectoryPlanningResult:
        """Return a safe result instead of an invalid command trajectory."""
        ...
