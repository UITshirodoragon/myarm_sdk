"""Deterministic placeholder kinematics implementation."""

from myarm_sdk.core import (
    IKRequest,
    IKResult,
    JointPositions,
    Pose,
    SingularityMetrics,
)


class IdentityKinematicsAdapter:
    """Placeholder used only for wiring and contract tests."""

    def forward(self, joints: JointPositions) -> Pose:
        values = joints.values
        return Pose(
            position=(values[0], values[1], values[2]),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

    def solve_ik(self, request: IKRequest) -> IKResult:
        solution = JointPositions(
            (
                request.target_pose.position[0],
                request.target_pose.position[1],
                request.target_pose.position[2],
            )
            + request.seed.values[3:]
        )
        return IKResult(
            q_solution=solution,
            converged=True,
            failure_reason=None,
            detail="identity placeholder solve",
            position_residual_m=0.0,
            orientation_residual_rad=0.0,
            iteration_count=1,
            singularity=SingularityMetrics(
                minimum_singular_value=1.0,
                condition_number=1.0,
                rank=6,
                near_singular=False,
                singular=False,
            ),
            seed=request.seed,
            active_joint_limits=(),
            minimum_joint_limit_margin_rad=float("inf"),
        )
