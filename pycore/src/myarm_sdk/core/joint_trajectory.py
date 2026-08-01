"""Validated, ROS-independent joint-space trajectory container."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple

from .trajectory_point import TrajectoryPoint


@dataclass(frozen=True)
class JointTrajectory:
    """Ordered joint-space points in one canonical joint convention.

    This type intentionally does not impose position, velocity or acceleration
    limits: those belong to the robot-specific :class:`JointMotionLimits`.
    It does guarantee a canonical joint order and a strictly increasing,
    zero-based time line.  The minimum-jerk planner produces points with
    velocity and acceleration populated; the container also accepts imported
    trajectories that do not carry derivatives yet.
    """

    joint_names: Tuple[str, ...]
    points: Tuple[TrajectoryPoint, ...]

    def __init__(
        self, joint_names: Sequence[str], points: Sequence[TrajectoryPoint]
    ) -> None:
        names = tuple(str(name) for name in joint_names)
        trajectory_points = tuple(points)
        if len(names) != 6:
            raise ValueError("MyArm M750 trajectory requires exactly 6 joint names")
        if any(not name.strip() for name in names):
            raise ValueError("joint_names must contain non-empty names")
        if len(set(names)) != len(names):
            raise ValueError("joint_names must be unique")
        if not trajectory_points:
            raise ValueError("trajectory must contain at least one point")
        if not all(isinstance(point, TrajectoryPoint) for point in trajectory_points):
            raise TypeError("trajectory points must be TrajectoryPoint values")
        if not math.isclose(
            trajectory_points[0].time_from_start_s,
            0.0,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            raise ValueError("first trajectory point must be scheduled at t=0")
        previous_time_s = trajectory_points[0].time_from_start_s
        for point in trajectory_points[1:]:
            if point.time_from_start_s <= previous_time_s:
                raise ValueError("trajectory timestamps must be strictly increasing")
            previous_time_s = point.time_from_start_s
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "points", trajectory_points)

    @property
    def duration_s(self) -> float:
        """Return the exact final ``time_from_start_s``."""
        return self.points[-1].time_from_start_s

    @property
    def has_derivatives(self) -> bool:
        """Return whether every point contains velocity and acceleration."""
        return all(
            point.velocities is not None and point.accelerations is not None
            for point in self.points
        )
