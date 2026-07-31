from typing import Protocol

from myarm_sdk.model import JointPositions, Pose


class Kinematics(Protocol):
    """Forward and inverse kinematics for one robot model."""

    def forward(self, joints: JointPositions) -> Pose:
        ...

    def inverse(self, pose: Pose, seed: JointPositions) -> JointPositions:
        ...
