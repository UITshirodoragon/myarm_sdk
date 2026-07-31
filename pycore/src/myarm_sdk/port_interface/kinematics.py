"""Forward and inverse kinematics contract."""

from typing import Protocol

from myarm_sdk.core import JointPositions, Pose


class KinematicsInterface(Protocol):
    """Forward and inverse kinematics for one robot model."""

    def forward(self, joints: JointPositions) -> Pose:
        ...

    def inverse(self, pose: Pose, seed: JointPositions) -> JointPositions:
        ...
