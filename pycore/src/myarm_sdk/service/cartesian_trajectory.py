"""Cartesian trajectory-planning capability for ROS and non-ROS callers."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Sequence, Tuple

from myarm_sdk.core.cartesian_trajectory_planning import (
    CartesianPathMode,
    CartesianTrajectoryPlanningRequest,
    CartesianTrajectoryPlanningResult,
    CartesianTrajectoryPolicy,
)
from myarm_sdk.core.configuration import load_sdk_yaml
from myarm_sdk.core.joint_metadata import JointMetadata
from myarm_sdk.core.joint_positions import JointPositions
from myarm_sdk.core.joint_trajectory_planning import (
    JointMotionLimits,
    TimeScalingPolicy,
)
from myarm_sdk.core.pose import Pose
from myarm_sdk.core.validation import require_enabled
from myarm_sdk.plugin_adapter.kinematics.pinocchio_kinematics import (
    PinocchioKinematicsAdapter,
)
from myarm_sdk.plugin_adapter.trajectory.cartesian_sequential_clik_trajectory_planner import (
    CartesianSequentialCLIKTrajectoryPlannerAdapter,
)
from myarm_sdk.port_interface.cartesian_trajectory import (
    CartesianTrajectoryPlannerInterface,
)


class CartesianTrajectoryPlannerService:
    """Expose pure sequential Cartesian planning without a feedback cache.

    The caller must always pass the newest validated measured ``q_start``.
    A configured service constructs its own Pinocchio adapter so the planner
    cannot race the stateful one-shot ``KinematicsService`` in another node.
    """

    def __init__(
        self,
        planner: CartesianTrajectoryPlannerInterface,
        motion_limits: JointMotionLimits,
        joint_names: Optional[Sequence[str]] = None,
        base_frame: str = "base_link",
        tool_frame: str = "tool0",
        default_policy: Optional[CartesianTrajectoryPolicy] = None,
    ) -> None:
        self._planner = planner
        self._motion_limits = motion_limits
        self._joint_names = tuple(joint_names or motion_limits.joint_names)
        if self._joint_names != motion_limits.joint_names:
            raise ValueError("joint_names must match motion_limits canonical order")
        self._base_frame = str(base_frame)
        self._tool_frame = str(tool_frame)
        if not self._base_frame or not self._tool_frame:
            raise ValueError("base_frame and tool_frame must be non-empty")
        self._default_policy = default_policy or CartesianTrajectoryPolicy()

    @classmethod
    def from_config(
        cls,
        service_config: Mapping[str, Any],
        kinematics_service_config: Mapping[str, Any],
        package_share_directory: Callable[[str], str],
        robot_config: Mapping[str, Any],
    ) -> CartesianTrajectoryPlannerService:
        """Build an isolated Cartesian Pinocchio planner from runtime config.

        The Cartesian adapter profile controls geometry, timing and validation.
        The selected kinematics profile remains the sole source of the URDF,
        base/tool frames, joint order and IK solver/damping policy.
        """
        require_enabled(service_config, "cartesian_trajectory_planner")
        if service_config.get("plugin_adapter") != "cartesian_sequential_clik":
            raise ValueError(
                "Only the cartesian_sequential_clik trajectory plugin adapter is available"
            )
        adapter_config = load_sdk_yaml(str(service_config["plugin_config"]))
        if adapter_config.get("plugin_adapter") != "cartesian_sequential_clik":
            raise ValueError(
                "Cartesian trajectory plugin config must select cartesian_sequential_clik"
            )
        kinematics, joint_metadata, ik_policy = cls._kinematics_from_config(
            kinematics_service_config,
            package_share_directory,
            robot_config,
        )
        motion_limits = JointMotionLimits(
            joint_metadata=joint_metadata,
            acceleration_limits_rad_s2=cls._acceleration_limits_from_profile(
                adapter_config, joint_metadata
            ),
        )
        default_policy = cls._policy_from_config(adapter_config, ik_policy)
        return cls(
            planner=CartesianSequentialCLIKTrajectoryPlannerAdapter(kinematics),
            motion_limits=motion_limits,
            joint_names=kinematics.joint_names,
            base_frame=kinematics.base_frame,
            tool_frame=kinematics.tool_frame,
            default_policy=default_policy,
        )

    @property
    def motion_limits(self) -> JointMotionLimits:
        """Return authoritative joint motion limits for this planner."""
        return self._motion_limits

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return self._joint_names

    @property
    def base_frame(self) -> str:
        return self._base_frame

    @property
    def tool_frame(self) -> str:
        return self._tool_frame

    @property
    def default_policy(self) -> CartesianTrajectoryPolicy:
        return self._default_policy

    def plan(
        self, request: CartesianTrajectoryPlanningRequest
    ) -> CartesianTrajectoryPlanningResult:
        """Plan a caller-constructed full request."""
        return self._planner.plan(request)

    def plan_cartesian_motion(
        self,
        q_start: JointPositions,
        target_pose: Pose,
        policy: Optional[CartesianTrajectoryPolicy] = None,
        time_scaling: Optional[TimeScalingPolicy] = None,
        motion_limits: Optional[JointMotionLimits] = None,
    ) -> CartesianTrajectoryPlanningResult:
        """Convenience form for callers with fresh feedback and a TCP target."""
        resolved_policy = policy or self._default_policy
        if time_scaling is not None:
            resolved_policy = replace(resolved_policy, time_scaling=time_scaling)
        return self.plan(
            CartesianTrajectoryPlanningRequest(
                q_start=q_start,
                target_pose=target_pose,
                motion_limits=motion_limits or self._motion_limits,
                policy=resolved_policy,
            )
        )

    @classmethod
    def _kinematics_from_config(
        cls,
        kinematics_service_config: Mapping[str, Any],
        package_share_directory: Callable[[str], str],
        robot_config: Mapping[str, Any],
    ) -> Tuple[PinocchioKinematicsAdapter, Tuple[JointMetadata, ...], Any]:
        require_enabled(kinematics_service_config, "kinematics")
        if kinematics_service_config.get("plugin_adapter") != "pinocchio":
            raise ValueError("Cartesian planning currently requires pinocchio kinematics")
        kinematics_config = load_sdk_yaml(
            str(kinematics_service_config["plugin_config"])
        )
        if kinematics_config.get("plugin_adapter") != "pinocchio":
            raise ValueError("Kinematics plugin config must select pinocchio")
        frames = cls._mapping(kinematics_config.get("frames"), "kinematics frames")
        joint_order = cls._mapping(
            kinematics_config.get("joint_order"), "kinematics joint_order"
        )
        if joint_order.get("source") != "urdf":
            raise ValueError("kinematics joint_order.source must be 'urdf'")
        joint_names = tuple(str(name) for name in joint_order["names"])
        if len(joint_names) != 6 or len(set(joint_names)) != 6:
            raise ValueError("kinematics joint_order must contain six unique joint names")
        adapter_robot_description = cls._mapping(
            kinematics_config.get("robot_description"), "kinematics robot_description"
        )
        robot_description = cls._mapping(
            robot_config.get("robot_description"), "robot.robot_description"
        )
        if (
            str(adapter_robot_description.get("package"))
            != str(robot_description.get("package"))
            or str(adapter_robot_description.get("relative_path"))
            != str(robot_description.get("relative_path"))
        ):
            raise ValueError(
                "kinematics adapter robot_description must match robot.robot_description"
            )
        shared_joint_order = cls._mapping(
            robot_config.get("joint_order"), "robot.joint_order"
        )
        if shared_joint_order.get("source") != "urdf":
            raise ValueError("robot.joint_order.source must be 'urdf'")
        if tuple(str(name) for name in shared_joint_order["names"]) != joint_names:
            raise ValueError(
                "robot.joint_order.names must match kinematics joint_order.names"
            )
        joint_convention = cls._mapping(
            kinematics_config.get("joint_convention"), "kinematics joint_convention"
        )
        if (
            joint_convention.get("positive_direction")
            != "right_hand_rule_about_urdf_axis"
        ):
            raise ValueError("kinematics positive direction must follow URDF")
        policy = PinocchioKinematicsAdapter.policy_from_config(
            cls._mapping(kinematics_config.get("solver"), "kinematics solver"),
            cls._mapping(
                kinematics_config.get("joint_limits"), "kinematics joint_limits"
            ),
        )
        description_share = Path(
            package_share_directory(str(robot_description["package"]))
        )
        kinematics = PinocchioKinematicsAdapter(
            urdf_path=description_share / str(robot_description["relative_path"]),
            joint_names=joint_names,
            base_frame=str(frames["base"]),
            tool_frame=str(frames["tool"]),
            default_policy=policy,
        )
        return kinematics, kinematics.joint_metadata, policy

    @classmethod
    def _policy_from_config(
        cls, adapter_config: Mapping[str, Any], ik_policy: Any
    ) -> CartesianTrajectoryPolicy:
        path_config = cls._mapping(adapter_config.get("path"), "cartesian path")
        validation_config = cls._mapping(
            adapter_config.get("validation"), "cartesian validation"
        )
        timing_config = cls._mapping(
            adapter_config.get("time_scaling"), "cartesian time_scaling"
        )
        time_scaling = TimeScalingPolicy(
            mode=timing_config.get("mode", TimeScalingPolicy().mode.value),
            requested_duration_s=timing_config.get("requested_duration_s"),
            speed_scale=timing_config.get("speed_scale"),
            sample_period_s=float(timing_config["sample_period_s"]),
        )
        return CartesianTrajectoryPolicy(
            path_mode=CartesianPathMode(str(path_config["mode"])),
            ik_policy=ik_policy,
            time_scaling=time_scaling,
            max_translation_step_m=float(path_config["max_translation_step_m"]),
            max_rotation_step_rad=float(path_config["max_rotation_step_rad"]),
            max_waypoints=int(path_config["max_waypoints"]),
            maximum_joint_step_rad=float(path_config["maximum_joint_step_rad"]),
            dense_validation_sample_period_s=float(
                validation_config["dense_sample_period_s"]
            ),
            position_validation_tolerance_m=float(
                validation_config["position_tolerance_m"]
            ),
            orientation_validation_tolerance_rad=float(
                validation_config["orientation_tolerance_rad"]
            ),
            max_duration_stretch_iterations=int(
                validation_config.get("max_duration_stretch_iterations", 8)
            ),
        )

    @classmethod
    def _acceleration_limits_from_profile(
        cls,
        adapter_config: Mapping[str, Any],
        joint_metadata: Sequence[JointMetadata],
    ) -> Sequence[float]:
        profile_path = adapter_config.get("joint_motion_limits_config")
        if not isinstance(profile_path, str) or not profile_path:
            raise ValueError(
                "cartesian joint_motion_limits_config must reference a joint profile"
            )
        profile = load_sdk_yaml(profile_path)
        return cls._acceleration_limits(
            profile.get("acceleration_limits_rad_s2"), joint_metadata
        )

    @staticmethod
    def _acceleration_limits(
        value: Any, joint_metadata: Sequence[JointMetadata]
    ) -> Sequence[float]:
        if not isinstance(value, dict):
            raise TypeError("acceleration_limits_rad_s2 must be a mapping")
        names = tuple(metadata.name for metadata in joint_metadata)
        missing = tuple(name for name in names if name not in value)
        unexpected = tuple(name for name in value if name not in names)
        if missing or unexpected:
            details = []
            if missing:
                details.append("missing {}".format(", ".join(missing)))
            if unexpected:
                details.append("unexpected {}".format(", ".join(unexpected)))
            raise ValueError(
                "acceleration_limits_rad_s2 names must match joint metadata: {}".format(
                    "; ".join(details)
                )
            )
        return tuple(float(value[name]) for name in names)

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value
