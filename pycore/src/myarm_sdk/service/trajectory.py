"""Trajectory capability placeholder."""

from typing import Sequence

from myarm_sdk.core import JointPositions, TrajectoryPoint
from myarm_sdk.port_interface import TrajectoryInterface


class TrajectoryService:
    """Own a trajectory interface for a future trajectory ROS node."""

    def __init__(self, trajectory: TrajectoryInterface) -> None:
        self._trajectory = trajectory

    def plan(
        self, start: JointPositions, target: JointPositions, duration_s: float
    ) -> Sequence[TrajectoryPoint]:
        return self._trajectory.plan(start, target, duration_s)
