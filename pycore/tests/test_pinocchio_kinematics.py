from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pinocchio")

from myarm_sdk.core import (
    IKFailureReason,
    IKNearSingularityPolicy,
    IKPolicy,
    IKRequest,
    IKSeedSource,
    IKTaskMode,
    JointPositions,
    Pose,
    load_sdk_yaml,
)
from myarm_sdk.core.cartesian_trajectory_planning import (
    CartesianTrajectoryPlanningRequest,
    CartesianTrajectoryPolicy,
    pose_orientation_distance_rad,
    pose_translation_distance_m,
)
from myarm_sdk.core.joint_trajectory_planning import (
    JointMotionLimits,
    TimeScalingMode,
    TimeScalingPolicy,
)
from myarm_sdk.plugin_adapter.kinematics import PinocchioKinematicsAdapter
from myarm_sdk.plugin_adapter.robot_arm.myarm_m750_robot_arm import (
    MyArmM750RobotArm,
)
from myarm_sdk.service import KinematicsService
from myarm_sdk.plugin_adapter.trajectory import (
    CartesianSequentialCLIKTrajectoryPlannerAdapter,
)

PROJECT_ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = (
    PROJECT_ROOT / "ros2_ws/src/myarm_description/urdf/myarm_m750_poe_v3_2.urdf"
)
HOME = JointPositions((0.0, -0.35, 0.70, 0.0, -0.35, 0.0))


@pytest.fixture(scope="module")
def adapter():
    return PinocchioKinematicsAdapter(URDF_PATH)


def _policy(**changes):
    return replace(IKPolicy(max_solve_time_ms=500.0), **changes)


def test_urdf_is_the_source_of_truth_for_order_axes_limits_and_frames(adapter):
    assert adapter.joint_names == (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    )
    assert tuple(metadata.axis_xyz for metadata in adapter.joint_metadata) == (
        (0.0, 0.0, 1.0),
        (0.0, 1.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (1.0, 0.0, 0.0),
    )
    assert adapter.joint_metadata[0].lower_limit_rad == pytest.approx(-2.879793265790644)
    assert adapter.joint_metadata[4].upper_limit_rad == pytest.approx(2.0943951023931953)
    assert all(
        metadata.positive_direction == "right_hand_rule_about_urdf_axis"
        for metadata in adapter.joint_metadata
    )
    assert adapter.base_frame == "base_link"
    assert adapter.tool_frame == "tool0"


def test_fk_to_ik_to_fk_converges_from_home_with_residuals_and_limits(adapter):
    expected_joints = JointPositions((0.20, -0.40, 0.80, 0.50, 0.50, -0.80))
    target_pose = adapter.forward(expected_joints)

    result = adapter.solve_ik(IKRequest(target_pose, HOME, _policy()))

    assert result.converged is True
    assert result.q_solution is not None
    assert result.failure_reason is None
    assert result.position_residual_m <= 0.001
    assert result.orientation_residual_rad <= 0.02
    assert result.iteration_count > 0
    assert adapter.joint_limit_violations(result.q_solution) == ()
    actual_pose = adapter.forward(result.q_solution)
    assert np.linalg.norm(
        np.asarray(actual_pose.position) - np.asarray(target_pose.position)
    ) <= 0.001


def test_fk_to_cartesian_plan_to_fk_uses_the_real_pinocchio_chain(adapter):
    q_start = HOME
    q_goal = JointPositions((0.02, -0.34, 0.69, 0.0, -0.34, 0.0))
    target_pose = adapter.forward(q_goal)
    policy = CartesianTrajectoryPolicy(
        ik_policy=_policy(),
        time_scaling=TimeScalingPolicy(
            mode=TimeScalingMode.AUTO_LIMITED,
            sample_period_s=0.05,
        ),
    )
    result = CartesianSequentialCLIKTrajectoryPlannerAdapter(adapter).plan(
        CartesianTrajectoryPlanningRequest(
            q_start=q_start,
            target_pose=target_pose,
            motion_limits=JointMotionLimits(
                adapter.joint_metadata,
                acceleration_limits_rad_s2=(0.5,) * 6,
            ),
            policy=policy,
        )
    )

    assert result.succeeded is True
    assert result.trajectory is not None
    assert result.trajectory.has_derivatives
    actual_pose = adapter.forward(result.trajectory.points[-1].positions)
    assert pose_translation_distance_m(actual_pose, target_pose) <= 0.003
    assert pose_orientation_distance_rad(actual_pose, target_pose) <= 0.05


def test_near_limit_target_is_kept_inside_hard_and_software_limits(adapter):
    q1_upper = adapter.joint_metadata[0].upper_limit_rad
    near_limit = JointPositions((q1_upper - 0.03, -0.35, 0.70, 0.0, -0.35, 0.0))
    result = adapter.solve_ik(
        IKRequest(adapter.forward(near_limit), near_limit, _policy())
    )

    assert result.converged is True
    assert result.q_solution is not None
    assert result.minimum_joint_limit_margin_rad >= 0.02
    assert adapter.joint_limit_violations(result.q_solution) == ()


def test_seed_outside_hard_limit_and_software_margin_block_are_reported(adapter):
    q1_upper = adapter.joint_metadata[0].upper_limit_rad
    out_of_limit_seed = JointPositions((q1_upper + 0.01, -0.35, 0.70, 0.0, -0.35, 0.0))
    seed_result = adapter.solve_ik(
        IKRequest(adapter.forward(HOME), out_of_limit_seed, _policy())
    )
    assert seed_result.converged is False
    assert seed_result.failure_reason is IKFailureReason.SEED_OUT_OF_LIMIT

    target_outside_software_margin = JointPositions(
        (q1_upper - 0.01, -0.35, 0.70, 0.0, -0.35, 0.0)
    )
    blocked_result = adapter.solve_ik(
        IKRequest(
            adapter.forward(target_outside_software_margin),
            target_outside_software_margin,
            _policy(),
        )
    )
    assert blocked_result.converged is False
    assert blocked_result.failure_reason is IKFailureReason.JOINT_LIMIT_BLOCKED


def test_unreachable_target_and_timeout_are_classified(adapter):
    unreachable = adapter.solve_ik(
        IKRequest(
            Pose(position=(2.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0)),
            HOME,
            _policy(),
        )
    )
    assert unreachable.converged is False
    assert unreachable.failure_reason is IKFailureReason.UNREACHABLE

    reachable_pose = adapter.forward(
        JointPositions((0.20, -0.40, 0.80, 0.50, 0.50, -0.80))
    )
    timeout = adapter.solve_ik(
        IKRequest(reachable_pose, HOME, _policy(max_solve_time_ms=0.000001))
    )
    assert timeout.converged is False
    assert timeout.failure_reason is IKFailureReason.TIMEOUT


def test_q5_zero_is_reported_as_singular_and_position_only_is_explicit(adapter):
    wrist_singular = JointPositions((0.0, -0.35, 0.70, 0.0, 0.0, 0.0))
    singular_result = adapter.solve_ik(
        IKRequest(adapter.forward(wrist_singular), wrist_singular, _policy())
    )
    assert singular_result.converged is False
    assert singular_result.failure_reason is IKFailureReason.SINGULAR
    assert singular_result.singularity.near_singular is True

    wrist_near_singular = JointPositions((0.0, -0.35, 0.70, 0.0, 0.001, 0.0))
    near_result = adapter.solve_ik(
        IKRequest(
            adapter.forward(wrist_near_singular), wrist_near_singular, _policy()
        )
    )
    assert near_result.converged is False
    assert near_result.failure_reason is IKFailureReason.NEAR_SINGULAR
    assert near_result.singularity.near_singular is True

    source = JointPositions((0.20, -0.40, 0.80, 0.50, 0.50, -0.80))
    source_pose = adapter.forward(source)
    position_only_target = Pose(
        position=source_pose.position,
        orientation=(0.0, 0.0, 0.0, 1.0),
    )
    position_only = adapter.solve_ik(
        IKRequest(
            position_only_target,
            HOME,
            _policy(
                task_mode=IKTaskMode.POSITION_ONLY,
                near_singularity_policy=IKNearSingularityPolicy.WARN,
            ),
        )
    )
    assert position_only.converged is True
    assert position_only.q_solution is not None
    assert position_only.position_residual_m <= 0.001
    assert position_only.orientation_residual_rad > 0.02


def test_robot_adapter_calibration_converts_hardware_feedback_to_model_space():
    adapter = object.__new__(MyArmM750RobotArm)
    adapter._model_to_hardware_offsets_rad = (  # pylint: disable=protected-access
        MyArmM750RobotArm.DEFAULT_MODEL_TO_HARDWARE_OFFSETS_RAD
    )
    model = JointPositions((0.1, -0.2, 0.3, -0.4, 0.5, -0.6))

    hardware = adapter.hardware_from_model_positions(model)
    recovered = adapter.model_from_hardware_positions(hardware)

    assert recovered == model
    assert hardware[1] == pytest.approx(model.values[1] + np.deg2rad(10.0))
    assert hardware[2] == pytest.approx(model.values[2] - np.deg2rad(10.0))


def test_configured_service_uses_fresh_measured_model_state_as_real_ik_seed(adapter):
    config = load_sdk_yaml("service/config/services.yaml")
    service_config = config["services"]["kinematics"]
    service = KinematicsService.from_config(
        service_config,
        lambda package: str(PROJECT_ROOT / "ros2_ws/src" / package),
        robot_config=config["robot"],
    )
    target = adapter.forward(JointPositions((0.20, -0.40, 0.80, 0.50, 0.50, -0.80)))
    service.set_target_pose(target)

    missing_feedback = service.step(now_monotonic_s=20.0)
    assert missing_feedback.ik_result is not None
    assert missing_feedback.ik_result.failure_reason is IKFailureReason.SEED_UNAVAILABLE

    service.update_measured_joint_positions(HOME, received_at_monotonic_s=20.1)
    service.set_target_pose(target)
    solved = service.step(now_monotonic_s=20.2)
    assert solved.seed_source is IKSeedSource.MEASURED_JOINT_STATE
    assert solved.ik_result is not None and solved.ik_result.converged is True
    assert solved.command_updated is True
