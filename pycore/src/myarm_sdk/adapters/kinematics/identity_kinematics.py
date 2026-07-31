"""Deterministic placeholder for wiring and contract tests.

It intentionally does not model the physical M750. Replace it with a solver
adapter (for example Pinocchio) in applications that require real kinematics.
"""

from myarm_sdk.model import JointPositions, Pose


class IdentityKinematics:
    def forward(self, joints: JointPositions) -> Pose:
        values = joints.values
        return Pose(position=(values[0], values[1], values[2]), orientation=(0, 0, 0, 1))

    def inverse(self, pose: Pose, seed: JointPositions) -> JointPositions:
        return JointPositions((pose.position[0], pose.position[1], pose.position[2]) + seed.values[3:])
