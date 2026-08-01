"""Value types and policies for validated joint-space trajectory planning."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Sequence, Tuple

from .joint_metadata import JointMetadata
from .joint_positions import JointPositions
from .joint_trajectory import JointTrajectory


class TimeScalingMode(str, Enum):
    """How the planner resolves a common duration for all joints."""

    AUTO_LIMITED = "auto_limited"
    REQUESTED_DURATION_STRETCH = "requested_duration_stretch"
    REQUESTED_DURATION_STRICT = "requested_duration_strict"
    SPEED_SCALE = "speed_scale"


class TrajectoryPlanningFailureReason(str, Enum):
    """Machine-readable reason why a trajectory was not produced."""

    START_OUT_OF_LIMIT = "start_out_of_limit"
    GOAL_OUT_OF_LIMIT = "goal_out_of_limit"
    DURATION_BELOW_LIMIT = "duration_below_limit"
    TRAJECTORY_VALIDATION_FAILED = "trajectory_validation_failed"


@dataclass(frozen=True)
class JointMotionLimits:
    """Canonical hard position, velocity and acceleration limits.

    Position and velocity values come from the selected URDF's
    :class:`JointMetadata`; acceleration values are explicit because URDF does
    not define them for this robot model.
    """

    joint_metadata: Tuple[JointMetadata, ...]
    acceleration_limits_rad_s2: Tuple[float, ...]

    def __init__(
        self,
        joint_metadata: Sequence[JointMetadata],
        acceleration_limits_rad_s2: Sequence[float],
    ) -> None:
        metadata = tuple(joint_metadata)
        acceleration_limits = tuple(
            float(limit) for limit in acceleration_limits_rad_s2
        )
        if len(metadata) != 6:
            raise ValueError("MyArm M750 motion limits require exactly 6 joints")
        if len(acceleration_limits) != len(metadata):
            raise ValueError(
                "acceleration_limits_rad_s2 length must match joint metadata"
            )
        names = tuple(metadata_item.name for metadata_item in metadata)
        if any(not name.strip() for name in names) or len(set(names)) != len(names):
            raise ValueError("joint metadata must have unique, non-empty names")
        for metadata_item in metadata:
            if not all(
                math.isfinite(value)
                for value in (
                    metadata_item.lower_limit_rad,
                    metadata_item.upper_limit_rad,
                    metadata_item.velocity_limit_rad_s,
                )
            ):
                raise ValueError("joint metadata limits must be finite")
            if metadata_item.lower_limit_rad >= metadata_item.upper_limit_rad:
                raise ValueError(
                    "joint metadata lower limit must be below upper limit"
                )
            if metadata_item.velocity_limit_rad_s <= 0.0:
                raise ValueError("joint metadata velocity limit must be positive")
        if not all(
            math.isfinite(limit) and limit > 0.0 for limit in acceleration_limits
        ):
            raise ValueError("acceleration limits must be finite and positive")
        object.__setattr__(self, "joint_metadata", metadata)
        object.__setattr__(self, "acceleration_limits_rad_s2", acceleration_limits)

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """Return the canonical ordered names shared by all motion values."""
        return tuple(metadata_item.name for metadata_item in self.joint_metadata)

    @property
    def velocity_limits_rad_s(self) -> Tuple[float, ...]:
        """Return URDF velocity limits in canonical joint order."""
        return tuple(
            metadata_item.velocity_limit_rad_s
            for metadata_item in self.joint_metadata
        )

    def position_violations(self, joints: JointPositions) -> Tuple[str, ...]:
        """Return hard-limit violations for one canonical joint vector."""
        violations = []
        for metadata_item, value in zip(self.joint_metadata, joints.values):
            if value < metadata_item.lower_limit_rad or value > metadata_item.upper_limit_rad:
                violations.append(
                    f"{metadata_item.name}={value:.9f} outside "
                    f"[{metadata_item.lower_limit_rad:.9f}, "
                    f"{metadata_item.upper_limit_rad:.9f}]"
                )
        return tuple(violations)

    def trajectory_violations(
        self, trajectory: JointTrajectory, require_derivatives: bool = True
    ) -> Tuple[str, ...]:
        """Validate every trajectory point against hard motion limits."""
        if trajectory.joint_names != self.joint_names:
            return ("trajectory joint_names do not match motion limits",)
        violations = []
        tolerance = 1e-10
        for index, point in enumerate(trajectory.points):
            for position_error in self.position_violations(point.positions):
                violations.append(f"point {index}: {position_error}")
            if point.velocities is None or point.accelerations is None:
                if require_derivatives:
                    violations.append(
                        f"point {index}: velocity and acceleration are required"
                    )
                continue
            for joint_index, metadata_item in enumerate(self.joint_metadata):
                velocity = abs(point.velocities.values[joint_index])
                if velocity > metadata_item.velocity_limit_rad_s + tolerance:
                    violations.append(
                        f"point {index}: {metadata_item.name} velocity "
                        f"{velocity:.9f} exceeds "
                        f"{metadata_item.velocity_limit_rad_s:.9f}"
                    )
                acceleration = abs(point.accelerations.values[joint_index])
                acceleration_limit = self.acceleration_limits_rad_s2[joint_index]
                if acceleration > acceleration_limit + tolerance:
                    violations.append(
                        f"point {index}: {metadata_item.name} acceleration "
                        f"{acceleration:.9f} exceeds {acceleration_limit:.9f}"
                    )
        return tuple(violations)


@dataclass(frozen=True)
class TimeScalingPolicy:
    """Duration and sampling policy for a synchronous joint move.

    ``REQUESTED_DURATION_STRETCH`` is the default: a supplied duration is
    honoured only when it meets every limit, otherwise it is increased.  With
    no requested duration it resolves to the automatically limited duration.

    ``SPEED_SCALE`` applies ``0 < r <= 1`` after resolving its base duration:
    ``T = T_base / r``.  Consequently velocity scales with ``r`` and
    acceleration with ``r²``.
    """

    mode: TimeScalingMode = TimeScalingMode.REQUESTED_DURATION_STRETCH
    requested_duration_s: Optional[float] = None
    speed_scale: Optional[float] = None
    sample_period_s: float = 0.2

    def __post_init__(self) -> None:
        mode = TimeScalingMode(self.mode)
        requested_duration_s = self._optional_positive(
            self.requested_duration_s, "requested_duration_s"
        )
        speed_scale = self._optional_positive(self.speed_scale, "speed_scale")
        sample_period_s = float(self.sample_period_s)
        if not math.isfinite(sample_period_s) or sample_period_s <= 0.0:
            raise ValueError("sample_period_s must be finite and positive")
        if mode is TimeScalingMode.REQUESTED_DURATION_STRICT:
            if requested_duration_s is None:
                raise ValueError(
                    "requested_duration_s is required for requested_duration_strict"
                )
        elif mode is TimeScalingMode.SPEED_SCALE:
            if speed_scale is None or speed_scale > 1.0:
                raise ValueError("speed_scale mode requires 0 < speed_scale <= 1")
        elif speed_scale is not None:
            raise ValueError("speed_scale is only valid when mode is speed_scale")
        object.__setattr__(self, "mode", mode)
        object.__setattr__(self, "requested_duration_s", requested_duration_s)
        object.__setattr__(self, "speed_scale", speed_scale)
        object.__setattr__(self, "sample_period_s", sample_period_s)

    @staticmethod
    def _optional_positive(value: Optional[float], name: str) -> Optional[float]:
        if value is None:
            return None
        normalized = float(value)
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return normalized


@dataclass(frozen=True)
class TrajectoryPlanningRequest:
    """One request from a fresh measured state to a joint-space goal.

    ``q_start`` must be the caller's newest validated measured joint state for
    physical execution.  The planner is pure and deliberately does not own a
    robot feedback cache or a clock.
    """

    q_start: JointPositions
    q_goal: JointPositions
    motion_limits: JointMotionLimits
    time_scaling: TimeScalingPolicy = TimeScalingPolicy()


@dataclass(frozen=True)
class TrajectoryPlanningResult:
    """Safe result of planning; failures never contain a trajectory."""

    trajectory: Optional[JointTrajectory]
    succeeded: bool
    failure_reason: Optional[TrajectoryPlanningFailureReason]
    detail: str
    requested_duration_s: Optional[float]
    minimum_duration_s: float
    resolved_duration_s: Optional[float]
    duration_adjusted: bool

    def __post_init__(self) -> None:
        minimum_duration_s = float(self.minimum_duration_s)
        if not math.isfinite(minimum_duration_s) or minimum_duration_s < 0.0:
            raise ValueError("minimum_duration_s must be finite and non-negative")
        object.__setattr__(self, "minimum_duration_s", minimum_duration_s)
        if self.succeeded:
            if self.trajectory is None or self.failure_reason is not None:
                raise ValueError("successful planning result must contain only a trajectory")
            if self.resolved_duration_s is None:
                raise ValueError("successful planning result requires resolved duration")
            resolved_duration_s = float(self.resolved_duration_s)
            if not math.isfinite(resolved_duration_s) or resolved_duration_s <= 0.0:
                raise ValueError("resolved_duration_s must be finite and positive")
            if not math.isclose(
                self.trajectory.duration_s,
                resolved_duration_s,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise ValueError(
                    "trajectory final timestamp must equal resolved_duration_s"
                )
            object.__setattr__(self, "resolved_duration_s", resolved_duration_s)
        elif self.trajectory is not None or self.failure_reason is None:
            raise ValueError("failed planning result must contain a failure reason only")

    @property
    def success(self) -> bool:
        """Compatibility-friendly spelling for consumers using ``success``."""
        return self.succeeded
