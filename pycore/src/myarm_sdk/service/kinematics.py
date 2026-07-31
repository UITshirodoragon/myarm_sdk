"""Stateful kinematics service used by the MyArm Cartesian command node."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Tuple

from myarm_sdk.core import JointPositions, Pose, load_sdk_yaml
from myarm_sdk.core.validation import require_enabled
from myarm_sdk.plugin_adapter.kinematics import (
    InverseKinematicsError,
    PinocchioKinematicsAdapter,
)
from myarm_sdk.port_interface import KinematicsInterface


class KinematicsServiceError(RuntimeError):
    """A kinematics operation could not be completed by the configured backend."""


@dataclass(frozen=True)
class KinematicsStep:
    """The joint target and verified TCP pose for one 5 Hz service cycle."""

    joint_positions: JointPositions
    tcp_pose: Pose
    target_active: bool


class KinematicsService:
    """Manage target pose, IK seed and FK verification for one MyArm model."""

    def __init__(
        self,
        kinematics: KinematicsInterface,
        joint_names: Tuple[str, ...],
        base_frame: str,
        initial_joint_positions: JointPositions,
    ) -> None:
        self._kinematics = kinematics
        self._joint_names = joint_names
        self._base_frame = base_frame
        self._joint_positions = initial_joint_positions
        self._target_pose: Optional[Pose] = None

    @classmethod
    def from_config(
        cls,
        service_config: Mapping[str, Any],
        package_share_directory: Callable[[str], str],
    ) -> KinematicsService:
        """Create the configured Pinocchio service using an injected ROS path resolver."""
        require_enabled(service_config, "kinematics")
        if service_config.get("plugin_adapter") != "pinocchio":
            raise ValueError("Only the pinocchio kinematics plugin adapter is available")

        adapter_config = load_sdk_yaml(str(service_config["plugin_config"]))
        if adapter_config.get("plugin_adapter") != "pinocchio":
            raise ValueError("Kinematics plugin config must select pinocchio")

        solver = adapter_config["solver"]
        frames = adapter_config["frames"]
        robot_description = adapter_config["robot_description"]
        description_share = Path(
            package_share_directory(str(robot_description["package"]))
        )
        urdf_path = description_share / str(robot_description["relative_path"])
        kinematics = PinocchioKinematicsAdapter(
            urdf_path=urdf_path,
            tool_frame=str(frames["tool"]),
            max_iterations=int(solver["max_iterations"]),
            position_tolerance_m=float(solver["position_tolerance_m"]),
            orientation_tolerance_rad=float(solver["orientation_tolerance_rad"]),
            damping=float(solver["damping"]),
            step_size=float(solver["step_size"]),
        )

        named_poses = service_config["named_poses"]
        initial_pose_name = str(service_config["initial_named_pose"])
        try:
            initial_values = named_poses[initial_pose_name]["positions_rad"]
        except KeyError as error:
            raise ValueError(
                f"Unknown initial_named_pose '{initial_pose_name}'"
            ) from error

        return cls(
            kinematics=kinematics,
            joint_names=tuple(str(name) for name in adapter_config["joint_names"]),
            base_frame=str(frames["base"]),
            initial_joint_positions=JointPositions(initial_values),
        )

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return self._joint_names

    @property
    def base_frame(self) -> str:
        return self._base_frame

    def set_target_pose(self, pose: Pose) -> None:
        """Store the latest desired TCP pose; it will be solved on the next tick."""
        self._target_pose = pose

    def clear_target_pose(self) -> None:
        """Stop retrying an invalid target while preserving the last valid state."""
        self._target_pose = None

    def step(self) -> KinematicsStep:
        """Advance one deterministic service cycle, solving IK only when commanded."""
        target_active = self._target_pose is not None
        if target_active:
            try:
                self._joint_positions = self._kinematics.inverse(
                    self._target_pose, self._joint_positions
                )
            except InverseKinematicsError as error:
                raise KinematicsServiceError(str(error)) from error
        return KinematicsStep(
            joint_positions=self._joint_positions,
            tcp_pose=self._kinematics.forward(self._joint_positions),
            target_active=target_active,
        )
