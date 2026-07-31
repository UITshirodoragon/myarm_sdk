"""Pinocchio FK/IK adapter for the canonical MyArm M750 PoE URDF."""

from pathlib import Path
from typing import Sequence

import numpy as np

from myarm_sdk.core import JointPositions, Pose
from myarm_sdk.core.spatial import (
    quaternion_xyzw_from_rotation_matrix,
    rotation_matrix_from_quaternion_xyzw,
)


class InverseKinematicsError(RuntimeError):
    """The numerical IK solver could not reach a valid target pose."""


class PinocchioKinematicsAdapter:
    """Compute FK and damped-least-squares IK for the six M750 arm joints."""

    ARM_JOINT_NAMES = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    )

    def __init__(
        self,
        urdf_path: Path,
        tool_frame: str = "tool0",
        max_iterations: int = 100,
        position_tolerance_m: float = 0.001,
        orientation_tolerance_rad: float = 0.02,
        damping: float = 0.001,
        step_size: float = 0.5,
    ) -> None:
        if not urdf_path.is_file():
            raise ValueError(f"Pinocchio URDF file does not exist: {urdf_path}")
        if max_iterations < 1:
            raise ValueError("max_iterations must be positive")
        if position_tolerance_m <= 0.0 or orientation_tolerance_rad <= 0.0:
            raise ValueError("IK tolerances must be positive")
        if damping <= 0.0 or not 0.0 < step_size <= 1.0:
            raise ValueError("damping must be positive and step_size must be in (0, 1]")

        try:
            import pinocchio as pin
        except ImportError as error:
            raise RuntimeError(
                "Install kinematics support with `pip install myarm-sdk[kinematics]`."
            ) from error

        self._pin = pin
        full_model = pin.buildModelFromUrdf(str(urdf_path))
        unknown_joints = [
            name for name in self.ARM_JOINT_NAMES if not full_model.existJointName(name)
        ]
        if unknown_joints:
            raise ValueError("URDF is missing arm joints: {}".format(", ".join(unknown_joints)))

        locked_joint_ids = [
            joint_id
            for joint_id, name in enumerate(full_model.names)
            if joint_id != 0 and name not in self.ARM_JOINT_NAMES
        ]
        self._model = pin.buildReducedModel(
            full_model, locked_joint_ids, pin.neutral(full_model)
        )
        if self._model.nq != len(self.ARM_JOINT_NAMES):
            raise ValueError(
                f"reduced Pinocchio model must have six joint coordinates, got {self._model.nq}"
            )
        if tool_frame not in [frame.name for frame in self._model.frames]:
            raise ValueError(f"URDF is missing tool frame: {tool_frame}")

        self._data = self._model.createData()
        self._tool_frame = tool_frame
        self._tool_frame_id = self._model.getFrameId(tool_frame)
        self._max_iterations = max_iterations
        self._position_tolerance_m = position_tolerance_m
        self._orientation_tolerance_rad = orientation_tolerance_rad
        self._damping = damping
        self._step_size = step_size

    def forward(self, joints: JointPositions) -> Pose:
        configuration = self._as_configuration(joints.values)
        self._pin.forwardKinematics(self._model, self._data, configuration)
        self._pin.updateFramePlacements(self._model, self._data)
        placement = self._data.oMf[self._tool_frame_id]
        return Pose(
            position=tuple(float(value) for value in placement.translation),
            orientation=quaternion_xyzw_from_rotation_matrix(placement.rotation),
        )

    def inverse(self, pose: Pose, seed: JointPositions) -> JointPositions:
        desired = self._pin.SE3(
            rotation_matrix_from_quaternion_xyzw(pose.orientation),
            np.asarray(pose.position, dtype=float),
        )
        if not np.isfinite(desired.translation).all():
            raise ValueError("pose position must contain finite values")

        configuration = self._clamp(self._as_configuration(seed.values))
        for _ in range(self._max_iterations):
            self._pin.forwardKinematics(self._model, self._data, configuration)
            self._pin.updateFramePlacements(self._model, self._data)
            current = self._data.oMf[self._tool_frame_id]
            error_transform = current.actInv(desired)
            error = self._pin.log6(error_transform).vector
            if (
                float(np.linalg.norm(error[:3])) <= self._position_tolerance_m
                and float(np.linalg.norm(error[3:])) <= self._orientation_tolerance_rad
            ):
                return JointPositions(configuration)

            frame_jacobian = self._pin.computeFrameJacobian(
                self._model,
                self._data,
                configuration,
                self._tool_frame_id,
                self._pin.ReferenceFrame.LOCAL,
            )
            task_jacobian = self._pin.Jlog6(error_transform.inverse()) @ frame_jacobian
            system = task_jacobian @ task_jacobian.T + self._damping * np.eye(6)
            velocity = task_jacobian.T @ np.linalg.solve(system, error)
            configuration = self._clamp(
                self._pin.integrate(
                    self._model, configuration, self._step_size * velocity
                )
            )

        raise InverseKinematicsError(
            f"IK did not converge for {self._tool_frame} after {self._max_iterations} iterations"
        )

    def _as_configuration(self, values: Sequence[float]) -> np.ndarray:
        configuration = np.asarray(values, dtype=float)
        if configuration.shape != (len(self.ARM_JOINT_NAMES),):
            raise ValueError("MyArm M750 requires exactly six joint values")
        if not np.isfinite(configuration).all():
            raise ValueError("joint values must be finite")
        return configuration

    def _clamp(self, configuration: np.ndarray) -> np.ndarray:
        return np.minimum(
            np.maximum(configuration, self._model.lowerPositionLimit),
            self._model.upperPositionLimit,
        )
