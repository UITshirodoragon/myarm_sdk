"""Focused checks for sequential Cartesian-to-joint trajectory planning."""

from __future__ import annotations

import math

import pytest
from myarm_sdk.core.cartesian_trajectory_planning import (
    CartesianPathMode,
    CartesianTrajectoryPlanningFailureReason,
    CartesianTrajectoryPlanningRequest,
    CartesianTrajectoryPolicy,
    interpolate_cartesian_pose,
    make_cartesian_reference_path,
    pose_orientation_distance_rad,
)
from myarm_sdk.core.ik import (
    IKFailureReason,
    IKNearSingularityPolicy,
    IKPolicy,
    IKResult,
    IKTaskMode,
    SingularityMetrics,
)
from myarm_sdk.core.joint_metadata import JointMetadata
from myarm_sdk.core.joint_positions import JointPositions
from myarm_sdk.core.joint_trajectory_interpolation import (
    dense_trajectory_violations,
    sample_joint_trajectory,
)
from myarm_sdk.core.joint_trajectory_planning import (
    JointMotionLimits,
    TimeScalingMode,
    TimeScalingPolicy,
)
from myarm_sdk.core.pose import Pose
from myarm_sdk.plugin_adapter.trajectory.cartesian_sequential_clik_trajectory_planner import (
    CartesianSequentialCLIKTrajectoryPlannerAdapter,
)


def _motion_limits(
    velocity_limit_rad_s=1.0,
    acceleration_limit_rad_s2=1.0,
    lower_limit_rad=-2.0,
    upper_limit_rad=2.0,
):
    metadata = tuple(
        JointMetadata(
            name=f"joint_{index + 1}",
            axis_xyz=(0.0, 0.0, 1.0),
            lower_limit_rad=lower_limit_rad,
            upper_limit_rad=upper_limit_rad,
            velocity_limit_rad_s=velocity_limit_rad_s,
        )
        for index in range(6)
    )
    return JointMotionLimits(metadata, (acceleration_limit_rad_s2,) * 6)


def _metrics():
    return SingularityMetrics(
        minimum_singular_value=0.5,
        condition_number=2.0,
        rank=6,
        near_singular=False,
        singular=False,
    )


class _CartesianFakeKinematics:
    """Identity XYZ kinematics with deterministic sequential IK behavior."""

    def __init__(
        self,
        unreachable_after_x=None,
        jump_solution=False,
        singularity=None,
        position_residual_m=0.0,
        orientation_residual_rad=0.0,
        solutions=None,
    ):
        self.requests = []
        self.unreachable_after_x = unreachable_after_x
        self.jump_solution = jump_solution
        self.singularity = singularity or _metrics()
        self.position_residual_m = position_residual_m
        self.orientation_residual_rad = orientation_residual_rad
        self.solutions = tuple(solutions or ())
        self._solution_index = 0

    def forward(self, joints):
        return Pose(
            position=(joints.values[0], joints.values[1], joints.values[2]),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

    def solve_ik(self, request):
        self.requests.append(request)
        if (
            self.unreachable_after_x is not None
            and request.target_pose.position[0] > self.unreachable_after_x
        ):
            return IKResult(
                q_solution=None,
                converged=False,
                failure_reason=IKFailureReason.UNREACHABLE,
                detail="fake unreachable target",
                position_residual_m=0.25,
                orientation_residual_rad=0.0,
                iteration_count=2,
                singularity=_metrics(),
                seed=request.seed,
                active_joint_limits=(),
                minimum_joint_limit_margin_rad=1.0,
            )
        if self.solutions:
            solution = self.solutions[min(self._solution_index, len(self.solutions) - 1)]
            self._solution_index += 1
        elif self.jump_solution:
            solution = JointPositions((1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        else:
            solution = JointPositions(
                request.target_pose.position + request.seed.values[3:]
            )
        return IKResult(
            q_solution=solution,
            converged=True,
            failure_reason=None,
            detail="fake converged",
            position_residual_m=self.position_residual_m,
            orientation_residual_rad=self.orientation_residual_rad,
            iteration_count=1,
            singularity=self.singularity,
            seed=request.seed,
            active_joint_limits=(),
            minimum_joint_limit_margin_rad=1.0,
        )


class _MaskedAxisFakeKinematics(_CartesianFakeKinematics):
    """Inject an FK Z error to exercise component-wise dense validation."""

    def forward(self, joints):
        return Pose(
            position=(joints.values[0], joints.values[1], joints.values[2] + 0.1),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )


def _policy(**overrides):
    values = {
        "ik_policy": IKPolicy(),
        "time_scaling": TimeScalingPolicy(
            mode=TimeScalingMode.AUTO_LIMITED,
            sample_period_s=0.05,
        ),
        "max_translation_step_m": 0.01,
        "max_rotation_step_rad": 0.1,
        "maximum_joint_step_rad": 0.2,
        "dense_validation_sample_period_s": 0.01,
        "position_validation_tolerance_m": 0.01,
        "orientation_validation_tolerance_rad": 0.01,
    }
    values.update(overrides)
    return CartesianTrajectoryPolicy(**values)


def _request(policy=None, target=None, q_start=None, motion_limits=None):
    return CartesianTrajectoryPlanningRequest(
        q_start=q_start or JointPositions((0.0,) * 6),
        target_pose=target
        or Pose(position=(0.04, 0.02, 0.0), orientation=(0.0, 0.0, 0.0, 1.0)),
        motion_limits=motion_limits or _motion_limits(),
        policy=policy or _policy(),
    )


def test_sequential_clik_plan_has_complete_derivatives_and_sequential_seeds():
    kinematics = _CartesianFakeKinematics()
    request = _request()

    result = CartesianSequentialCLIKTrajectoryPlannerAdapter(kinematics).plan(request)

    assert result.succeeded
    assert result.trajectory is not None
    assert result.trajectory.has_derivatives
    assert result.waypoint_count == 6
    assert result.trajectory.points[0].positions == request.q_start
    assert result.trajectory.points[-1].positions == JointPositions(
        (0.04, 0.02, 0.0, 0.0, 0.0, 0.0)
    )
    assert result.trajectory.points[0].velocities == JointPositions((0.0,) * 6)
    assert result.trajectory.points[-1].velocities == JointPositions((0.0,) * 6)
    assert result.trajectory.points[0].accelerations == JointPositions((0.0,) * 6)
    assert result.trajectory.points[-1].accelerations == JointPositions((0.0,) * 6)
    assert kinematics.requests[0].seed == request.q_start
    assert all(
        following.seed.values[:3] == preceding.target_pose.position
        for preceding, following in zip(kinematics.requests, kinematics.requests[1:])
    )
    assert not dense_trajectory_violations(
        result.trajectory, request.motion_limits, sample_period_s=0.005
    )
    midpoint = sample_joint_trajectory(
        result.trajectory, result.trajectory.duration_s * 0.5
    )
    assert midpoint.positions.values[0] == pytest.approx(0.02, abs=0.01)


def test_unreachable_waypoint_returns_no_partial_trajectory():
    kinematics = _CartesianFakeKinematics(unreachable_after_x=0.02)

    result = CartesianSequentialCLIKTrajectoryPlannerAdapter(kinematics).plan(
        _request()
    )

    assert not result.succeeded
    assert result.trajectory is None
    assert (
        result.failure_reason
        is CartesianTrajectoryPlanningFailureReason.UNREACHABLE
    )
    assert result.failed_waypoint_index is not None
    assert len(result.reference_path) == 6


def test_branch_discontinuity_is_rejected_before_command_trajectory_exists():
    result = CartesianSequentialCLIKTrajectoryPlannerAdapter(
        _CartesianFakeKinematics(jump_solution=True)
    ).plan(_request())

    assert not result.succeeded
    assert result.trajectory is None
    assert (
        result.failure_reason
        is CartesianTrajectoryPlanningFailureReason.BRANCH_DISCONTINUITY
    )


def test_strict_requested_duration_rejects_instead_of_stretching():
    policy = _policy(
        time_scaling=TimeScalingPolicy(
            mode=TimeScalingMode.REQUESTED_DURATION_STRICT,
            requested_duration_s=0.01,
            sample_period_s=0.05,
        )
    )
    result = CartesianSequentialCLIKTrajectoryPlannerAdapter(
        _CartesianFakeKinematics()
    ).plan(_request(policy=policy))

    assert not result.succeeded
    assert result.trajectory is None
    assert (
        result.failure_reason
        is CartesianTrajectoryPlanningFailureReason.DURATION_BELOW_LIMIT
    )


def test_position_only_plan_does_not_reject_unconstrained_orientation_error():
    policy = _policy(
        ik_policy=IKPolicy(task_mode=IKTaskMode.POSITION_ONLY),
        # The fake IK keeps identity orientation while this target rotates 90°.
        orientation_validation_tolerance_rad=0.01,
    )
    target = Pose(
        position=(0.04, 0.02, 0.0),
        orientation=(0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    )

    result = CartesianSequentialCLIKTrajectoryPlannerAdapter(
        _CartesianFakeKinematics()
    ).plan(_request(policy=policy, target=target))

    assert result.succeeded
    assert result.trajectory is not None
    assert result.maximum_orientation_residual_rad > 1.0


def test_start_inside_hard_limits_but_outside_software_margin_is_rejected():
    result = CartesianSequentialCLIKTrajectoryPlannerAdapter(
        _CartesianFakeKinematics()
    ).plan(
        _request(
            policy=_policy(ik_policy=IKPolicy(safety_limit_margin_rad=0.02)),
            q_start=JointPositions((1.99, 0.0, 0.0, 0.0, 0.0, 0.0)),
        )
    )

    assert not result.succeeded
    assert result.trajectory is None
    assert (
        result.failure_reason
        is CartesianTrajectoryPlanningFailureReason.JOINT_LIMIT_BLOCKED
    )


def test_backend_claiming_success_with_bad_residual_or_singularity_is_rejected():
    high_residual = CartesianSequentialCLIKTrajectoryPlannerAdapter(
        _CartesianFakeKinematics(position_residual_m=0.02)
    ).plan(_request())
    assert not high_residual.succeeded
    assert high_residual.failure_reason is CartesianTrajectoryPlanningFailureReason.IK_FAILED

    near_singular_metrics = SingularityMetrics(
        minimum_singular_value=0.0005,
        condition_number=2000.0,
        rank=6,
        near_singular=True,
        singular=False,
    )
    near_singular = CartesianSequentialCLIKTrajectoryPlannerAdapter(
        _CartesianFakeKinematics(singularity=near_singular_metrics)
    ).plan(
        _request(
            policy=_policy(
                ik_policy=IKPolicy(
                    near_singularity_policy=IKNearSingularityPolicy.REJECT
                )
            )
        )
    )
    assert not near_singular.succeeded
    assert (
        near_singular.failure_reason
        is CartesianTrajectoryPlanningFailureReason.NEAR_SINGULAR
    )


def test_dense_quintic_cannot_enter_software_limit_margin_between_waypoints():
    # Each IK waypoint stays within the 0.02-rad software margin of the hard
    # upper bound 1.0, but the C2 quintic between them overshoots to ~0.989.
    planner = CartesianSequentialCLIKTrajectoryPlannerAdapter(
        _CartesianFakeKinematics(
            solutions=(
                JointPositions((0.97, 0.0, 0.0, 0.0, 0.0, 0.0)),
                JointPositions((0.65, 0.0, 0.0, 0.0, 0.0, 0.0)),
            )
        )
    )
    policy = _policy(
        ik_policy=IKPolicy(safety_limit_margin_rad=0.02),
        max_translation_step_m=0.15,
        maximum_joint_step_rad=0.35,
        dense_validation_sample_period_s=0.001,
        position_validation_tolerance_m=0.5,
    )
    result = planner.plan(
        _request(
            policy=policy,
            q_start=JointPositions((0.94, 0.0, 0.0, 0.0, 0.0, 0.0)),
            target=Pose(
                position=(0.65, 0.0, 0.0),
                orientation=(0.0, 0.0, 0.0, 1.0),
            ),
            motion_limits=_motion_limits(
                lower_limit_rad=-1.0,
                upper_limit_rad=1.0,
            ),
        )
    )

    assert not result.succeeded
    assert result.trajectory is None
    assert (
        result.failure_reason
        is CartesianTrajectoryPlanningFailureReason.JOINT_LIMIT_BLOCKED
    )
    assert result.minimum_joint_limit_margin_rad < 0.02


def test_dense_cartesian_validation_honours_individual_position_mask_axes():
    policy = _policy(
        ik_policy=IKPolicy(position_mask=(True, True, False)),
        maximum_joint_step_rad=0.2,
        position_validation_tolerance_m=0.01,
    )
    result = CartesianSequentialCLIKTrajectoryPlannerAdapter(
        _MaskedAxisFakeKinematics()
    ).plan(
        _request(
            policy=policy,
            target=Pose(
                position=(0.04, 0.02, 0.1),
                orientation=(0.0, 0.0, 0.0, 1.0),
            ),
        )
    )

    assert result.succeeded
    assert result.trajectory is not None
    assert result.maximum_position_residual_m >= 0.1


def test_linear_translation_slerp_reference_is_straight_and_se3_reaches_endpoint():
    start = Pose(
        position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0)
    )
    target = Pose(
        position=(0.2, 0.1, 0.0),
        orientation=(0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)),
    )
    linear_path = make_cartesian_reference_path(
        start,
        target,
        _policy(max_translation_step_m=0.05),
    )
    assert linear_path[0] == start
    assert linear_path[-1].position == pytest.approx(target.position)
    assert pose_orientation_distance_rad(linear_path[-1], target) == pytest.approx(0.0)
    # Every linear-translation sample stays on y = 0.5x for this target.
    assert all(pose.position[1] == pytest.approx(0.5 * pose.position[0]) for pose in linear_path)

    geodesic_endpoint = interpolate_cartesian_pose(
        start, target, 1.0, CartesianPathMode.SE3_GEODESIC
    )
    assert geodesic_endpoint.position == pytest.approx(target.position)
    assert pose_orientation_distance_rad(geodesic_endpoint, target) == pytest.approx(0.0)
