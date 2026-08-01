"""Cartesian-to-joint trajectory-planning contract."""

from typing import Protocol

from myarm_sdk.core.cartesian_trajectory_planning import (
    CartesianTrajectoryPlanningRequest,
    CartesianTrajectoryPlanningResult,
)


class CartesianTrajectoryPlannerInterface(Protocol):
    """Plan a complete validated Cartesian path without robot transport I/O."""

    def plan(
        self, request: CartesianTrajectoryPlanningRequest
    ) -> CartesianTrajectoryPlanningResult:
        """Return a trajectory only when every Cartesian waypoint is safe."""
        ...
