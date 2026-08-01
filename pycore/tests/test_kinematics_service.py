from myarm_sdk.core import (
    IKFailureReason,
    IKPolicy,
    IKRequest,
    IKResult,
    IKSeedPolicy,
    IKSeedSource,
    JointPositions,
    Pose,
    SingularityMetrics,
    load_sdk_yaml,
)
from myarm_sdk.service.kinematics import KinematicsService


class FakeKinematics:
    def __init__(self, should_fail=False):
        self.requests = []
        self.should_fail = should_fail

    def forward(self, joints):
        return Pose(
            position=(joints.values[0], joints.values[1], joints.values[2]),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

    def solve_ik(self, request):
        self.requests.append(request)
        if self.should_fail:
            return IKResult(
                q_solution=None,
                converged=False,
                failure_reason=IKFailureReason.UNREACHABLE,
                detail="target is unreachable",
                position_residual_m=1.0,
                orientation_residual_rad=1.0,
                iteration_count=3,
                singularity=_metrics(),
                seed=request.seed,
                active_joint_limits=(),
                minimum_joint_limit_margin_rad=0.5,
            )
        solution = JointPositions(
            request.target_pose.position + request.seed.values[3:]
        )
        return IKResult(
            q_solution=solution,
            converged=True,
            failure_reason=None,
            detail="fake solve",
            position_residual_m=0.0,
            orientation_residual_rad=0.0,
            iteration_count=1,
            singularity=_metrics(),
            seed=request.seed,
            active_joint_limits=(),
            minimum_joint_limit_margin_rad=0.5,
        )

    def joint_limit_violations(self, joints):
        return ()


def _metrics():
    return SingularityMetrics(
        minimum_singular_value=1.0,
        condition_number=1.0,
        rank=6,
        near_singular=False,
        singular=False,
    )


def _service(kinematics, seed_policy):
    return KinematicsService(
        kinematics=kinematics,
        joint_names=("j1", "j2", "j3", "j4", "j5", "j6"),
        base_frame="base_link",
        tool_frame="tool0",
        initial_joint_positions=JointPositions((0.0, -0.35, 0.70, 0.0, -0.35, 0.0)),
        default_ik_policy=IKPolicy(),
        seed_policy=seed_policy,
    )


def test_service_keeps_home_as_seed_without_publishing_an_implicit_goal():
    kinematics = FakeKinematics()
    service = _service(
        kinematics,
        IKSeedPolicy(source=IKSeedSource.LAST_COMMANDED),
    )

    initial = service.step(now_monotonic_s=1.0)
    assert initial.command_updated is False
    assert initial.joint_goal is None
    assert initial.commanded_joint_positions.values == (0.0, -0.35, 0.70, 0.0, -0.35, 0.0)

    service.set_target_pose(
        Pose(position=(0.1, 0.2, 0.3), orientation=(0.0, 0.0, 0.0, 1.0))
    )
    target = service.step(now_monotonic_s=1.1)

    assert target.target_processed is True
    assert target.command_updated is True
    assert target.joint_goal == target.commanded_joint_positions
    assert target.seed_source is IKSeedSource.LAST_COMMANDED
    assert kinematics.requests[-1].seed == initial.commanded_joint_positions
    assert target.ik_result is not None and target.ik_result.converged is True


def test_service_requires_fresh_measured_seed_and_never_replaces_safe_command_on_failure():
    failing_kinematics = FakeKinematics(should_fail=True)
    service = _service(
        failing_kinematics,
        IKSeedPolicy(
            source=IKSeedSource.MEASURED_JOINT_STATE,
            measured_state_max_age_s=0.5,
            allow_last_commanded_fallback=False,
        ),
    )
    initial = service.step(now_monotonic_s=10.0)
    service.set_target_pose(
        Pose(position=(0.1, 0.2, 0.3), orientation=(0.0, 0.0, 0.0, 1.0))
    )

    missing_feedback = service.step(now_monotonic_s=10.1)
    assert missing_feedback.command_updated is False
    assert missing_feedback.ik_result is not None
    assert missing_feedback.ik_result.failure_reason is IKFailureReason.SEED_UNAVAILABLE

    measured = JointPositions((0.2, -0.3, 0.6, 0.1, -0.4, 0.2))
    service.update_measured_joint_positions(measured, received_at_monotonic_s=10.2)
    service.set_target_pose(
        Pose(position=(0.3, 0.2, 0.1), orientation=(0.0, 0.0, 0.0, 1.0))
    )
    failed = service.step(now_monotonic_s=10.3)

    assert failed.seed_source is IKSeedSource.MEASURED_JOINT_STATE
    assert failing_kinematics.requests[-1].seed == measured
    assert failed.command_updated is False
    assert failed.commanded_joint_positions == initial.commanded_joint_positions
    assert failed.ik_result is not None
    assert failed.ik_result.failure_reason is IKFailureReason.UNREACHABLE


def test_service_accepts_explicit_complete_ik_request():
    kinematics = FakeKinematics()
    service = _service(
        kinematics,
        IKSeedPolicy(source=IKSeedSource.MEASURED_JOINT_STATE),
    )
    explicit_seed = JointPositions((0.2, -0.2, 0.5, 0.1, -0.4, 0.3))
    request = IKRequest(
        target_pose=Pose(
            position=(0.1, 0.2, 0.3), orientation=(0.0, 0.0, 0.0, 1.0)
        ),
        seed=explicit_seed,
        policy=IKPolicy(),
    )
    service.request_ik(request)

    step = service.step(now_monotonic_s=2.0)
    assert step.seed_source is IKSeedSource.EXPLICIT
    assert kinematics.requests[-1] == request
    assert step.ik_result is not None and step.ik_result.converged is True


def test_service_manifest_selects_home_and_real_measured_seed_policy():
    config = load_sdk_yaml("service/config/services.yaml")
    kinematics = config["services"]["kinematics"]

    assert kinematics["enabled"] is True
    assert kinematics["update_rate_hz"] == 5.0
    assert kinematics["initial_seed_named_pose"] == "home"
    assert kinematics["seed"]["source"] == "measured_joint_state"
    assert kinematics["seed"]["allow_last_commanded_fallback"] is False
