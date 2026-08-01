"""Validated quintic minimum-jerk planning in canonical joint space."""

from __future__ import annotations

import math
from typing import Optional

from myarm_sdk.core import (
    JointPositions,
    JointTrajectory,
    JointTrajectoryPlanningFailureReason,
    JointTrajectoryPlanningRequest,
    JointTrajectoryPlanningResult,
    TimeScalingMode,
    TrajectoryPoint,
)


class MinimumJerkJointTrajectoryPlannerAdapter:
    """Plan synchronized point-to-point motion with a quintic profile.

    The adapter is pure: it has no robot I/O, feedback cache or wall clock.
    Its caller supplies the fresh measured ``q_start`` in every request.
    """

    _MAX_NORMALIZED_VELOCITY = 15.0 / 8.0
    _MAX_NORMALIZED_ACCELERATION = 10.0 / math.sqrt(3.0)
    _FLOAT_TOLERANCE = 1e-12

    def plan(
        self, request: JointTrajectoryPlanningRequest
    ) -> JointTrajectoryPlanningResult:
        """Create a fully sampled trajectory or return a safe failure result."""
        start_violations = request.motion_limits.position_violations(request.q_start)
        if start_violations:
            return self._failure(
                request,
                JointTrajectoryPlanningFailureReason.START_OUT_OF_LIMIT,
                "q_start violates hard position limit: {}".format(
                    "; ".join(start_violations)
                ),
            )
        goal_violations = request.motion_limits.position_violations(request.q_goal)
        if goal_violations:
            return self._failure(
                request,
                JointTrajectoryPlanningFailureReason.GOAL_OUT_OF_LIMIT,
                "q_goal violates hard position limit: {}".format(
                    "; ".join(goal_violations)
                ),
            )

        minimum_duration_s = self.minimum_duration_s(request)
        resolved_duration_s = self._resolve_duration(request, minimum_duration_s)
        if resolved_duration_s is None:
            return self._failure(
                request,
                JointTrajectoryPlanningFailureReason.DURATION_BELOW_LIMIT,
                (
                    f"requested duration "
                    f"{request.time_scaling.requested_duration_s or 0.0:.9f}s "
                    f"is below minimum {minimum_duration_s:.9f}s for the "
                    "configured velocity and acceleration limits"
                ),
                minimum_duration_s,
            )

        trajectory = self._sample(request, resolved_duration_s)
        violations = request.motion_limits.trajectory_violations(trajectory)
        if violations:
            return self._failure(
                request,
                JointTrajectoryPlanningFailureReason.TRAJECTORY_VALIDATION_FAILED,
                "generated trajectory violates hard limits: {}".format(
                    "; ".join(violations)
                ),
                minimum_duration_s,
            )
        return JointTrajectoryPlanningResult(
            trajectory=trajectory,
            succeeded=True,
            failure_reason=None,
            detail="validated quintic minimum-jerk joint trajectory",
            requested_duration_s=request.time_scaling.requested_duration_s,
            minimum_duration_s=minimum_duration_s,
            resolved_duration_s=resolved_duration_s,
            duration_adjusted=self._duration_adjusted(request, resolved_duration_s),
        )

    def minimum_duration_s(
        self, request: JointTrajectoryPlanningRequest
    ) -> float:
        """Return the limit-safe common duration before policy time scaling.

        For a quintic minimum-jerk profile, ``max(s') = 15/8`` and
        ``max(abs(s'')) = 10/sqrt(3)``.  The exact per-joint bounds are:

        ``T_v = (15/8) * abs(delta_q) / velocity_limit`` and
        ``T_a = sqrt((10/sqrt(3)) * abs(delta_q) / acceleration_limit)``.
        """
        velocity_duration_s = 0.0
        acceleration_duration_s = 0.0
        for index, metadata in enumerate(request.motion_limits.joint_metadata):
            displacement = abs(
                request.q_goal.values[index] - request.q_start.values[index]
            )
            velocity_duration_s = max(
                velocity_duration_s,
                self._MAX_NORMALIZED_VELOCITY
                * displacement
                / metadata.velocity_limit_rad_s,
            )
            acceleration_duration_s = max(
                acceleration_duration_s,
                math.sqrt(
                    self._MAX_NORMALIZED_ACCELERATION
                    * displacement
                    / request.motion_limits.acceleration_limits_rad_s2[index]
                ),
            )
        # A stationary request still needs a positive timeline with t=0 and a
        # final point.  One sampling period is the smallest useful duration.
        return max(
            velocity_duration_s,
            acceleration_duration_s,
            request.time_scaling.sample_period_s,
        )

    def _resolve_duration(
        self, request: JointTrajectoryPlanningRequest, minimum_duration_s: float
    ) -> Optional[float]:
        policy = request.time_scaling
        requested_duration_s = policy.requested_duration_s
        if policy.mode is TimeScalingMode.AUTO_LIMITED:
            return minimum_duration_s
        if policy.mode is TimeScalingMode.REQUESTED_DURATION_STRETCH:
            return max(requested_duration_s or minimum_duration_s, minimum_duration_s)
        if policy.mode is TimeScalingMode.REQUESTED_DURATION_STRICT:
            if requested_duration_s is None:
                return None
            if requested_duration_s + self._FLOAT_TOLERANCE < minimum_duration_s:
                return None
            return requested_duration_s
        if policy.mode is TimeScalingMode.SPEED_SCALE:
            base_duration_s = max(
                requested_duration_s or minimum_duration_s, minimum_duration_s
            )
            # ``TimeScalingPolicy`` validates this value at construction.
            return base_duration_s / float(policy.speed_scale)
        raise ValueError(f"unsupported time scaling mode: {policy.mode}")

    def _sample(
        self, request: JointTrajectoryPlanningRequest, duration_s: float
    ) -> JointTrajectory:
        sample_period_s = request.time_scaling.sample_period_s
        intervals = max(1, math.ceil(duration_s / sample_period_s))
        displacement = tuple(
            goal - start
            for start, goal in zip(request.q_start.values, request.q_goal.values)
        )
        points = []
        for index in range(intervals + 1):
            normalized_time = index / intervals
            time_from_start_s = duration_s * normalized_time
            position_scale = self._position_scale(normalized_time)
            velocity_scale = self._velocity_scale(normalized_time) / duration_s
            acceleration_scale = (
                self._acceleration_scale(normalized_time) / (duration_s * duration_s)
            )
            points.append(
                TrajectoryPoint(
                    positions=JointPositions(
                        tuple(
                            start + delta * position_scale
                            for start, delta in zip(request.q_start.values, displacement)
                        )
                    ),
                    time_from_start_s=time_from_start_s,
                    velocities=JointPositions(
                        tuple(delta * velocity_scale for delta in displacement)
                    ),
                    accelerations=JointPositions(
                        tuple(delta * acceleration_scale for delta in displacement)
                    ),
                )
            )
        return JointTrajectory(request.motion_limits.joint_names, points)

    @staticmethod
    def _position_scale(normalized_time: float) -> float:
        if normalized_time <= 0.0:
            return 0.0
        if normalized_time >= 1.0:
            return 1.0
        squared = normalized_time * normalized_time
        cubed = squared * normalized_time
        value = (
            10.0 * cubed
            - 15.0 * cubed * normalized_time
            + 6.0 * cubed * squared
        )
        return max(0.0, min(1.0, value))

    @staticmethod
    def _velocity_scale(normalized_time: float) -> float:
        if normalized_time <= 0.0 or normalized_time >= 1.0:
            return 0.0
        squared = normalized_time * normalized_time
        return 30.0 * squared * (1.0 - normalized_time) * (1.0 - normalized_time)

    @staticmethod
    def _acceleration_scale(normalized_time: float) -> float:
        if normalized_time <= 0.0 or normalized_time >= 1.0:
            return 0.0
        squared = normalized_time * normalized_time
        cubed = squared * normalized_time
        return 60.0 * normalized_time - 180.0 * squared + 120.0 * cubed

    @staticmethod
    def _duration_adjusted(
        request: JointTrajectoryPlanningRequest, resolved_duration_s: float
    ) -> bool:
        requested_duration_s = request.time_scaling.requested_duration_s
        if requested_duration_s is None:
            return request.time_scaling.mode is TimeScalingMode.SPEED_SCALE
        return not math.isclose(
            requested_duration_s,
            resolved_duration_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        )

    @staticmethod
    def _failure(
        request: JointTrajectoryPlanningRequest,
        reason: JointTrajectoryPlanningFailureReason,
        detail: str,
        minimum_duration_s: float = 0.0,
    ) -> JointTrajectoryPlanningResult:
        return JointTrajectoryPlanningResult(
            trajectory=None,
            succeeded=False,
            failure_reason=reason,
            detail=detail,
            requested_duration_s=request.time_scaling.requested_duration_s,
            minimum_duration_s=minimum_duration_s,
            resolved_duration_s=None,
            duration_adjusted=False,
        )
