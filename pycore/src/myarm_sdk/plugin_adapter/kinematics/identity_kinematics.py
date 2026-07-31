"""Deterministic placeholder kinematics implementation."""

from myarm_sdk.core import JointPositions, Pose


class IdentityKinematicsAdapter:
    """Placeholder used only for wiring and contract tests."""

    def forward(self, joints: JointPositions) -> Pose:
        values = joints.values
        return Pose(
            position=(values[0], values[1], values[2]),
            orientation=(0.0, 0.0, 0.0, 1.0),
        )

    def inverse(self, pose: Pose, seed: JointPositions) -> JointPositions:
        return JointPositions(
            (pose.position[0], pose.position[1], pose.position[2]) + seed.values[3:]
        )
