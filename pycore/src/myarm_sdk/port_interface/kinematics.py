"""Forward and inverse kinematics contract."""

from typing import Protocol

from myarm_sdk.core import IKRequest, IKResult, JointPositions, Pose


class KinematicsInterface(Protocol):
    """Forward and inverse kinematics for one robot model."""

    def forward(self, joints: JointPositions) -> Pose:
        ...

    def solve_ik(self, request: IKRequest) -> IKResult:
        ...
