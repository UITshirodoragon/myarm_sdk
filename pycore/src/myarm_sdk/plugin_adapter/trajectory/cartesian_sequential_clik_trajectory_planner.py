"""Sequential CLIK Cartesian trajectory planner.

The adapter plans offline from one fresh measured ``q_start``.  It never owns
feedback, a clock or robot transport, and a failed waypoint never leaks a
partial command trajectory to its caller.
"""

from __future__ import annotations

import math
import threading
from typing import List, Optional, Sequence, Tuple

from myarm_sdk.core.cartesian_trajectory_planning import (
    CartesianTrajectoryPlanningFailureReason,
    CartesianTrajectoryPlanningRequest,
    CartesianTrajectoryPlanningResult,
    make_cartesian_reference_path,
    pose_orientation_distance_rad,
    pose_se3_error_vector,
    pose_translation_distance_m,
)
from myarm_sdk.core.ik import (
    IKFailureReason,
    IKNearSingularityPolicy,
    IKRequest,
    IKTaskMode,
)
from myarm_sdk.core.joint_positions import JointPositions
from myarm_sdk.core.joint_trajectory import JointTrajectory
from myarm_sdk.core.joint_trajectory_interpolation import (
    dense_sample_times,
    dense_trajectory_violations,
    minimum_jerk_position_scale,
    sample_joint_trajectory,
)
from myarm_sdk.core.joint_trajectory_planning import TimeScalingMode
from myarm_sdk.core.pose import Pose
from myarm_sdk.core.trajectory_point import TrajectoryPoint
from myarm_sdk.port_interface.kinematics import KinematicsInterface


class CartesianSequentialCLIKTrajectoryPlannerAdapter:
    """Plan Cartesian references by sequentially seeding each IK solve.

    A Pinocchio adapter owns mutable ``Data`` objects, therefore this adapter
    serializes ``forward`` and ``solve_ik`` calls.  Separate planner instances
    may still be used by separate ROS processes or workers.
    """

    _FLOAT_TOLERANCE = 1e-10

    def __init__(self, kinematics: KinematicsInterface) -> None:
        self._kinematics = kinematics
        self._plan_lock = threading.RLock()

    def plan(
        self, request: CartesianTrajectoryPlanningRequest
    ) -> CartesianTrajectoryPlanningResult:
        """Return one fully validated trajectory or an atomic failure result."""
        with self._plan_lock:
            return self._plan_locked(request)

    def _plan_locked(
        self, request: CartesianTrajectoryPlanningRequest
    ) -> CartesianTrajectoryPlanningResult:
        fallback_path = (request.target_pose, request.target_pose)
        start_violations = request.motion_limits.position_violations(request.q_start)
        if start_violations:
            return self._failure(
                request,
                fallback_path,
                CartesianTrajectoryPlanningFailureReason.START_OUT_OF_LIMIT,
                "q_start violates hard position limit: {}".format(
                    "; ".join(start_violations)
                ),
            )
        start_margin = self._joint_limit_margin(request, request.q_start)
        if (
            start_margin + self._FLOAT_TOLERANCE
            < request.policy.ik_policy.safety_limit_margin_rad
        ):
            return self._failure(
                request,
                fallback_path,
                CartesianTrajectoryPlanningFailureReason.JOINT_LIMIT_BLOCKED,
                (
                    "q_start violates the configured software joint-limit "
                    "margin"
                ),
                minimum_joint_limit_margin_rad=start_margin,
            )
        try:
            start_pose = self._kinematics.forward(request.q_start)
        except Exception as error:  # noqa: BLE001 - adapter faults are safe failures.
            return self._failure(
                request,
                fallback_path,
                CartesianTrajectoryPlanningFailureReason.FK_FAILED,
                f"FK(q_start) failed: {error}",
            )
        if not isinstance(start_pose, Pose):
            return self._failure(
                request,
                fallback_path,
                CartesianTrajectoryPlanningFailureReason.FK_FAILED,
                "FK(q_start) did not return a Pose",
            )
        try:
            reference_path = make_cartesian_reference_path(
                start_pose, request.target_pose, request.policy
            )
        except ValueError as error:
            reason = (
                CartesianTrajectoryPlanningFailureReason.WAYPOINT_LIMIT_EXCEEDED
                if "max_waypoints" in str(error)
                else CartesianTrajectoryPlanningFailureReason.INVALID_PATH
            )
            return self._failure(request, fallback_path, reason, str(error))

        joint_waypoints: List[JointPositions] = [request.q_start]
        maximum_position_residual_m = 0.0
        maximum_orientation_residual_rad = 0.0
        minimum_singular_value = float("nan")
        minimum_joint_limit_margin_rad = start_margin
        seed = request.q_start
        for index, target_pose in enumerate(reference_path[1:], start=1):
            try:
                ik_result = self._kinematics.solve_ik(
                    IKRequest(
                        target_pose=target_pose,
                        seed=seed,
                        policy=request.policy.ik_policy,
                    )
                )
            except Exception as error:  # noqa: BLE001 - no invalid command on backend error.
                return self._failure(
                    request,
                    reference_path,
                    CartesianTrajectoryPlanningFailureReason.IK_FAILED,
                    f"IK backend failed at waypoint {index}: {error}",
                    failed_waypoint_index=index,
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            maximum_position_residual_m = max(
                maximum_position_residual_m,
                self._finite_nonnegative(ik_result.position_residual_m),
            )
            maximum_orientation_residual_rad = max(
                maximum_orientation_residual_rad,
                self._finite_nonnegative(ik_result.orientation_residual_rad),
            )
            minimum_singular_value = self._minimum_finite(
                minimum_singular_value,
                ik_result.singularity.minimum_singular_value,
            )
            if not ik_result.converged or ik_result.q_solution is None:
                return self._failure(
                    request,
                    reference_path,
                    self._ik_failure_reason(ik_result.failure_reason),
                    f"IK rejected waypoint {index}: {ik_result.detail}",
                    failed_waypoint_index=index,
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            singularity_failure = self._singularity_failure_reason(
                ik_result.singularity,
                request,
            )
            if singularity_failure is not None:
                return self._failure(
                    request,
                    reference_path,
                    singularity_failure,
                    (
                        f"IK solution at waypoint {index} violates the "
                        "configured singularity policy"
                    ),
                    failed_waypoint_index=index,
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            residual_detail = self._residual_violation_detail(ik_result, request)
            if residual_detail is not None:
                return self._failure(
                    request,
                    reference_path,
                    CartesianTrajectoryPlanningFailureReason.IK_FAILED,
                    f"IK solution at waypoint {index} {residual_detail}",
                    failed_waypoint_index=index,
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            solution = ik_result.q_solution
            solution_violations = request.motion_limits.position_violations(solution)
            if solution_violations:
                return self._failure(
                    request,
                    reference_path,
                    CartesianTrajectoryPlanningFailureReason.JOINT_LIMIT_BLOCKED,
                    "IK solution at waypoint {} violates hard limits: {}".format(
                        index, "; ".join(solution_violations)
                    ),
                    failed_waypoint_index=index,
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            solution_margin = self._joint_limit_margin(request, solution)
            if (
                solution_margin + self._FLOAT_TOLERANCE
                < request.policy.ik_policy.safety_limit_margin_rad
            ):
                return self._failure(
                    request,
                    reference_path,
                    CartesianTrajectoryPlanningFailureReason.JOINT_LIMIT_BLOCKED,
                    (
                        f"IK solution at waypoint {index} violates the configured "
                        "software joint-limit margin"
                    ),
                    failed_waypoint_index=index,
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            maximum_joint_delta = max(
                abs(value - seed_value)
                for value, seed_value in zip(solution.values, seed.values)
            )
            if maximum_joint_delta > request.policy.maximum_joint_step_rad:
                return self._failure(
                    request,
                    reference_path,
                    CartesianTrajectoryPlanningFailureReason.BRANCH_DISCONTINUITY,
                    (
                        f"IK solution at waypoint {index} changes one joint by {maximum_joint_delta:.9f} rad, "
                        f"above maximum_joint_step_rad={request.policy.maximum_joint_step_rad:.9f}"
                    ),
                    failed_waypoint_index=index,
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            minimum_joint_limit_margin_rad = min(
                minimum_joint_limit_margin_rad,
                solution_margin,
            )
            joint_waypoints.append(solution)
            seed = solution

        normalized_velocities, normalized_accelerations = self._normalized_derivatives(
            joint_waypoints
        )
        minimum_duration_s = self._minimum_duration_s(
            request, normalized_velocities, normalized_accelerations
        )
        resolved_duration_s = self._resolve_duration(request, minimum_duration_s)
        if resolved_duration_s is None:
            return self._failure(
                request,
                reference_path,
                CartesianTrajectoryPlanningFailureReason.DURATION_BELOW_LIMIT,
                (
                    f"requested duration {request.policy.time_scaling.requested_duration_s or 0.0:.9f}s is below minimum {minimum_duration_s:.9f}s for "
                    "the configured joint limits"
                ),
                maximum_position_residual_m=maximum_position_residual_m,
                maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                minimum_singular_value=minimum_singular_value,
                minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                minimum_duration_s=minimum_duration_s,
            )

        for attempt in range(request.policy.max_duration_stretch_iterations + 1):
            trajectory = self._trajectory_from_waypoints(
                request, joint_waypoints, normalized_velocities,
                normalized_accelerations, resolved_duration_s
            )
            waypoint_violations = request.motion_limits.trajectory_violations(
                trajectory
            )
            dense_violations = dense_trajectory_violations(
                trajectory,
                request.motion_limits,
                request.policy.dense_validation_sample_period_s,
            )
            continuous_margin_rad, margin_violations = (
                self._continuous_software_margin_violations(request, trajectory)
            )
            minimum_joint_limit_margin_rad = min(
                minimum_joint_limit_margin_rad,
                continuous_margin_rad,
            )
            cartesian_validation = self._validate_cartesian_execution_path(
                request, start_pose, trajectory
            )
            maximum_position_residual_m = max(
                maximum_position_residual_m, cartesian_validation[0]
            )
            maximum_orientation_residual_rad = max(
                maximum_orientation_residual_rad, cartesian_validation[1]
            )
            if cartesian_validation[2] is not None:
                return self._failure(
                    request,
                    reference_path,
                    CartesianTrajectoryPlanningFailureReason.CARTESIAN_VALIDATION_FAILED,
                    cartesian_validation[2],
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            if margin_violations:
                return self._failure(
                    request,
                    reference_path,
                    CartesianTrajectoryPlanningFailureReason.JOINT_LIMIT_BLOCKED,
                    "trajectory enters the configured software joint-limit margin: {}".format(
                        "; ".join(margin_violations)
                    ),
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                    minimum_duration_s=minimum_duration_s,
                )
            if not waypoint_violations and not dense_violations:
                return CartesianTrajectoryPlanningResult(
                    trajectory=trajectory,
                    succeeded=True,
                    failure_reason=None,
                    detail="validated sequential CLIK Cartesian trajectory",
                    reference_path=reference_path,
                    requested_duration_s=(
                        request.policy.time_scaling.requested_duration_s
                    ),
                    minimum_duration_s=minimum_duration_s,
                    resolved_duration_s=resolved_duration_s,
                    duration_adjusted=self._duration_adjusted(
                        request, resolved_duration_s, minimum_duration_s
                    ),
                    failed_waypoint_index=None,
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                )
            if (
                request.policy.time_scaling.mode
                is TimeScalingMode.REQUESTED_DURATION_STRICT
                or attempt == request.policy.max_duration_stretch_iterations
            ):
                violations = waypoint_violations + dense_violations
                return self._failure(
                    request,
                    reference_path,
                    CartesianTrajectoryPlanningFailureReason.DENSE_VALIDATION_FAILED,
                    "trajectory violates hard limits: {}".format(
                        "; ".join(violations)
                    ),
                    maximum_position_residual_m=maximum_position_residual_m,
                    maximum_orientation_residual_rad=maximum_orientation_residual_rad,
                    minimum_singular_value=minimum_singular_value,
                    minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
                    minimum_duration_s=minimum_duration_s,
                )
            resolved_duration_s *= 2.0

        raise RuntimeError("Cartesian duration stretching loop terminated unexpectedly")

    @staticmethod
    def _normalized_derivatives(
        waypoints: Sequence[JointPositions],
    ) -> Tuple[Tuple[Tuple[float, ...], ...], Tuple[Tuple[float, ...], ...]]:
        """Estimate C2-compatible derivatives with zero endpoint motion."""
        segments = len(waypoints) - 1
        if segments < 1:
            raise ValueError("at least two joint waypoints are required")
        spacing = 1.0 / segments
        velocities: List[Tuple[float, ...]] = []
        accelerations: List[Tuple[float, ...]] = []
        zeros = (0.0,) * 6
        for index in range(len(waypoints)):
            if index == 0 or index == len(waypoints) - 1:
                velocities.append(zeros)
                accelerations.append(zeros)
                continue
            previous_values = waypoints[index - 1].values
            current_values = waypoints[index].values
            following_values = waypoints[index + 1].values
            velocities.append(
                tuple(
                    (following - previous) / (2.0 * spacing)
                    for previous, following in zip(previous_values, following_values)
                )
            )
            accelerations.append(
                tuple(
                    (following - 2.0 * current + previous) / (spacing * spacing)
                    for previous, current, following in zip(
                        previous_values, current_values, following_values
                    )
                )
            )
        return tuple(velocities), tuple(accelerations)

    @staticmethod
    def _minimum_duration_s(
        request: CartesianTrajectoryPlanningRequest,
        normalized_velocities: Sequence[Sequence[float]],
        normalized_accelerations: Sequence[Sequence[float]],
    ) -> float:
        velocity_duration_s = 0.0
        acceleration_duration_s = 0.0
        for index, metadata in enumerate(request.motion_limits.joint_metadata):
            maximum_normalized_velocity = max(
                abs(values[index]) for values in normalized_velocities
            )
            maximum_normalized_acceleration = max(
                abs(values[index]) for values in normalized_accelerations
            )
            velocity_duration_s = max(
                velocity_duration_s,
                maximum_normalized_velocity / metadata.velocity_limit_rad_s,
            )
            acceleration_duration_s = max(
                acceleration_duration_s,
                math.sqrt(
                    maximum_normalized_acceleration
                    / request.motion_limits.acceleration_limits_rad_s2[index]
                ),
            )
        segment_count = len(normalized_velocities) - 1
        return max(
            velocity_duration_s,
            acceleration_duration_s,
            segment_count * request.policy.time_scaling.sample_period_s,
        )

    @classmethod
    def _resolve_duration(
        cls,
        request: CartesianTrajectoryPlanningRequest,
        minimum_duration_s: float,
    ) -> Optional[float]:
        policy = request.policy.time_scaling
        requested_duration_s = policy.requested_duration_s
        if policy.mode is TimeScalingMode.AUTO_LIMITED:
            return minimum_duration_s
        if policy.mode is TimeScalingMode.REQUESTED_DURATION_STRETCH:
            return max(requested_duration_s or minimum_duration_s, minimum_duration_s)
        if policy.mode is TimeScalingMode.REQUESTED_DURATION_STRICT:
            if requested_duration_s is None:
                return None
            if requested_duration_s + cls._FLOAT_TOLERANCE < minimum_duration_s:
                return None
            return requested_duration_s
        if policy.mode is TimeScalingMode.SPEED_SCALE:
            base_duration_s = max(
                requested_duration_s or minimum_duration_s, minimum_duration_s
            )
            return base_duration_s / float(policy.speed_scale)
        raise ValueError(f"unsupported time scaling mode: {policy.mode}")

    @staticmethod
    def _trajectory_from_waypoints(
        request: CartesianTrajectoryPlanningRequest,
        waypoints: Sequence[JointPositions],
        normalized_velocities: Sequence[Sequence[float]],
        normalized_accelerations: Sequence[Sequence[float]],
        duration_s: float,
    ) -> JointTrajectory:
        segments = len(waypoints) - 1
        points = []
        for index, waypoint in enumerate(waypoints):
            points.append(
                TrajectoryPoint(
                    positions=waypoint,
                    time_from_start_s=duration_s * index / segments,
                    velocities=JointPositions(
                        tuple(value / duration_s for value in normalized_velocities[index])
                    ),
                    accelerations=JointPositions(
                        tuple(
                            value / (duration_s * duration_s)
                            for value in normalized_accelerations[index]
                        )
                    ),
                )
            )
        return JointTrajectory(request.motion_limits.joint_names, points)

    def _validate_cartesian_execution_path(
        self,
        request: CartesianTrajectoryPlanningRequest,
        start_pose: Pose,
        trajectory: JointTrajectory,
    ) -> Tuple[float, float, Optional[str]]:
        """FK-check the exact joint polynomial against the Cartesian reference."""
        maximum_position_error_m = 0.0
        maximum_orientation_error_rad = 0.0
        duration_s = trajectory.duration_s
        try:
            times = dense_sample_times(
                trajectory, request.policy.dense_validation_sample_period_s
            )
            for time_from_start_s in times:
                sample = sample_joint_trajectory(trajectory, time_from_start_s)
                actual_pose = self._kinematics.forward(sample.positions)
                if not isinstance(actual_pose, Pose):
                    return (
                        maximum_position_error_m,
                        maximum_orientation_error_rad,
                        "FK during dense validation did not return a Pose",
                    )
                desired_pose = self._desired_pose_at_time(
                    start_pose,
                    request.target_pose,
                    time_from_start_s / duration_s,
                    request,
                )
                position_error_m = pose_translation_distance_m(
                    desired_pose, actual_pose
                )
                orientation_error_rad = pose_orientation_distance_rad(
                    desired_pose, actual_pose
                )
                position_error_vector, orientation_error_vector = (
                    pose_se3_error_vector(actual_pose, desired_pose)
                )
                maximum_position_error_m = max(
                    maximum_position_error_m, position_error_m
                )
                maximum_orientation_error_rad = max(
                    maximum_orientation_error_rad, orientation_error_rad
                )
                position_mask = request.policy.ik_policy.position_mask
                orientation_mask = request.policy.ik_policy.orientation_mask
                position_invalid = (
                    any(position_mask)
                    and math.sqrt(
                        sum(
                            value * value
                            for value, enabled in zip(
                                position_error_vector, position_mask
                            )
                            if enabled
                        )
                    )
                    > request.policy.position_validation_tolerance_m
                )
                orientation_is_constrained = (
                    request.policy.ik_policy.task_mode is IKTaskMode.FULL_POSE
                    and any(orientation_mask)
                )
                orientation_invalid = (
                    orientation_is_constrained
                    and math.sqrt(
                        sum(
                            value * value
                            for value, enabled in zip(
                                orientation_error_vector, orientation_mask
                            )
                            if enabled
                        )
                    )
                    > request.policy.orientation_validation_tolerance_rad
                )
                if position_invalid or orientation_invalid:
                    return (
                        maximum_position_error_m,
                        maximum_orientation_error_rad,
                        (
                            f"FK validation at t={time_from_start_s:.9f}s has position error {position_error_m:.9f}m "
                            f"and orientation error {orientation_error_rad:.9f}rad"
                        ),
                    )
        except Exception as error:  # noqa: BLE001 - safety validation must fail closed.
            return (
                maximum_position_error_m,
                maximum_orientation_error_rad,
                f"FK during dense validation failed: {error}",
            )
        return maximum_position_error_m, maximum_orientation_error_rad, None

    def _continuous_software_margin_violations(
        self,
        request: CartesianTrajectoryPlanningRequest,
        trajectory: JointTrajectory,
    ) -> Tuple[float, Tuple[str, ...]]:
        """Check the exact executor polynomial against software safe bounds."""
        minimum_margin_rad = float("inf")
        violations = []
        required_margin_rad = request.policy.ik_policy.safety_limit_margin_rad
        for time_from_start_s in dense_sample_times(
            trajectory, request.policy.dense_validation_sample_period_s
        ):
            sample = sample_joint_trajectory(trajectory, time_from_start_s)
            margin_rad = self._joint_limit_margin(request, sample.positions)
            minimum_margin_rad = min(minimum_margin_rad, margin_rad)
            if margin_rad + self._FLOAT_TOLERANCE < required_margin_rad:
                violations.append(
                    "t={:.9f}s has margin {:.9f}rad below {:.9f}rad".format(
                        time_from_start_s,
                        margin_rad,
                        required_margin_rad,
                    )
                )
        return minimum_margin_rad, tuple(violations)

    @staticmethod
    def _desired_pose_at_time(
        start_pose: Pose,
        target_pose: Pose,
        normalized_time: float,
        request: CartesianTrajectoryPlanningRequest,
    ) -> Pose:
        from myarm_sdk.core.cartesian_trajectory_planning import (
            interpolate_cartesian_pose,
        )

        return interpolate_cartesian_pose(
            start_pose,
            target_pose,
            minimum_jerk_position_scale(normalized_time),
            request.policy.path_mode,
        )

    @staticmethod
    def _ik_failure_reason(
        reason: Optional[IKFailureReason],
    ) -> CartesianTrajectoryPlanningFailureReason:
        mapping = {
            IKFailureReason.UNREACHABLE: CartesianTrajectoryPlanningFailureReason.UNREACHABLE,
            IKFailureReason.JOINT_LIMIT_BLOCKED: CartesianTrajectoryPlanningFailureReason.JOINT_LIMIT_BLOCKED,
            IKFailureReason.SINGULAR: CartesianTrajectoryPlanningFailureReason.SINGULAR,
            IKFailureReason.NEAR_SINGULAR: CartesianTrajectoryPlanningFailureReason.NEAR_SINGULAR,
            IKFailureReason.TIMEOUT: CartesianTrajectoryPlanningFailureReason.IK_TIMEOUT,
            IKFailureReason.MAX_ITERATIONS: CartesianTrajectoryPlanningFailureReason.IK_MAX_ITERATIONS,
        }
        return mapping.get(reason, CartesianTrajectoryPlanningFailureReason.IK_FAILED)

    @staticmethod
    def _singularity_failure_reason(
        singularity,
        request: CartesianTrajectoryPlanningRequest,
    ) -> Optional[CartesianTrajectoryPlanningFailureReason]:
        """Defend against a backend reporting an inconsistent accepted IK result."""
        if singularity.singular:
            return CartesianTrajectoryPlanningFailureReason.SINGULAR
        if (
            singularity.near_singular
            and request.policy.ik_policy.near_singularity_policy
            is IKNearSingularityPolicy.REJECT
        ):
            return CartesianTrajectoryPlanningFailureReason.NEAR_SINGULAR
        return None

    @staticmethod
    def _residual_violation_detail(
        ik_result,
        request: CartesianTrajectoryPlanningRequest,
    ) -> Optional[str]:
        """Check reported active-task residuals independently of ``converged``."""
        policy = request.policy.ik_policy
        position_residual_m = float(ik_result.position_residual_m)
        orientation_residual_rad = float(ik_result.orientation_residual_rad)
        if not math.isfinite(position_residual_m) or not math.isfinite(
            orientation_residual_rad
        ):
            return "reports a non-finite residual"
        if (
            any(policy.position_mask)
            and position_residual_m > policy.position_tolerance_m
        ):
            return (
                f"position residual {position_residual_m:.9f}m exceeds "
                f"tolerance {policy.position_tolerance_m:.9f}m"
            )
        if (
            policy.task_mode is IKTaskMode.FULL_POSE
            and any(policy.orientation_mask)
            and orientation_residual_rad > policy.orientation_tolerance_rad
        ):
            return (
                f"orientation residual {orientation_residual_rad:.9f}rad exceeds "
                f"tolerance {policy.orientation_tolerance_rad:.9f}rad"
            )
        return None

    @staticmethod
    def _joint_limit_margin(
        request: CartesianTrajectoryPlanningRequest, joints: JointPositions
    ) -> float:
        return min(
            min(
                value - metadata.lower_limit_rad,
                metadata.upper_limit_rad - value,
            )
            for metadata, value in zip(request.motion_limits.joint_metadata, joints.values)
        )

    @staticmethod
    def _finite_nonnegative(value: float) -> float:
        normalized = float(value)
        if not math.isfinite(normalized):
            return 0.0
        return max(0.0, normalized)

    @staticmethod
    def _minimum_finite(current: float, candidate: float) -> float:
        normalized = float(candidate)
        if not math.isfinite(normalized):
            return current
        if not math.isfinite(current):
            return normalized
        return min(current, normalized)

    @staticmethod
    def _duration_adjusted(
        request: CartesianTrajectoryPlanningRequest,
        resolved_duration_s: float,
        minimum_duration_s: float,
    ) -> bool:
        requested_duration_s = request.policy.time_scaling.requested_duration_s
        if requested_duration_s is None:
            return (
                request.policy.time_scaling.mode is TimeScalingMode.SPEED_SCALE
                or resolved_duration_s
                > minimum_duration_s + CartesianSequentialCLIKTrajectoryPlannerAdapter._FLOAT_TOLERANCE
            )
        return not math.isclose(
            requested_duration_s,
            resolved_duration_s,
            rel_tol=0.0,
            abs_tol=1e-12,
        )

    @staticmethod
    def _failure(
        request: CartesianTrajectoryPlanningRequest,
        reference_path: Sequence[Pose],
        reason: CartesianTrajectoryPlanningFailureReason,
        detail: str,
        failed_waypoint_index: Optional[int] = None,
        maximum_position_residual_m: float = 0.0,
        maximum_orientation_residual_rad: float = 0.0,
        minimum_singular_value: float = float("nan"),
        minimum_joint_limit_margin_rad: float = float("nan"),
        minimum_duration_s: float = 0.0,
    ) -> CartesianTrajectoryPlanningResult:
        return CartesianTrajectoryPlanningResult(
            trajectory=None,
            succeeded=False,
            failure_reason=reason,
            detail=detail,
            reference_path=tuple(reference_path),
            requested_duration_s=request.policy.time_scaling.requested_duration_s,
            minimum_duration_s=minimum_duration_s,
            resolved_duration_s=None,
            duration_adjusted=False,
            failed_waypoint_index=failed_waypoint_index,
            maximum_position_residual_m=maximum_position_residual_m,
            maximum_orientation_residual_rad=maximum_orientation_residual_rad,
            minimum_singular_value=minimum_singular_value,
            minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
        )
