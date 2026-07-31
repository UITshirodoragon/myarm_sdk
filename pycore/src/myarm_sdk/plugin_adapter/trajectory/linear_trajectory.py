"""Simple linear interpolation in joint space."""

from __future__ import annotations

from typing import List, Sequence

from myarm_sdk.core import JointPositions, TrajectoryPoint


class LinearTrajectoryAdapter:
    """Generate evenly sampled, linear joint-space trajectories."""

    def __init__(self, sample_period_s: float = 0.2) -> None:
        if sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be positive")
        self._sample_period_s = sample_period_s

    def plan(
        self, start: JointPositions, target: JointPositions, duration_s: float
    ) -> Sequence[TrajectoryPoint]:
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        steps = max(1, round(duration_s / self._sample_period_s))
        points: List[TrajectoryPoint] = []
        for index in range(steps + 1):
            ratio = index / steps
            positions = JointPositions(
                tuple(
                    begin + (end - begin) * ratio
                    for begin, end in zip(start.values, target.values)
                )
            )
            points.append(TrajectoryPoint(positions, duration_s * ratio))
        return points
