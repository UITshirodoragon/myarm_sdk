"""Stateful kinematics service used by the MyArm Cartesian command node."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Tuple

from myarm_sdk.core import (
    IKFailureReason,
    IKPolicy,
    IKRequest,
    IKResult,
    IKSeedPolicy,
    IKSeedSource,
    JointMetadata,
    JointPositions,
    Pose,
    SingularityMetrics,
    load_sdk_yaml,
)
from myarm_sdk.core.validation import require_enabled
from myarm_sdk.plugin_adapter.kinematics import PinocchioKinematicsAdapter
from myarm_sdk.port_interface import KinematicsInterface


class KinematicsServiceError(RuntimeError):
    """A configuration or service-boundary operation could not be completed."""


@dataclass(frozen=True)
class KinematicsStep:
    """One 5 Hz service cycle with distinct command and measured state."""

    commanded_joint_positions: JointPositions
    # Present only for a successfully solved, explicitly requested target.
    # The configured home pose is an IK seed, never an implicit motion command.
    joint_goal: Optional[JointPositions]
    commanded_tcp_pose: Pose
    measured_joint_positions: Optional[JointPositions]
    measured_tcp_pose: Optional[Pose]
    measured_state_age_s: Optional[float]
    measured_state_fresh: bool
    target_processed: bool
    command_updated: bool
    ik_result: Optional[IKResult]
    seed_source: Optional[IKSeedSource]


@dataclass(frozen=True)
class _PendingTarget:
    target_pose: Pose
    policy: IKPolicy
    explicit_seed: Optional[JointPositions]


class KinematicsService:
    """Manage measured-state seeded IK and last-known-safe command state."""

    def __init__(
        self,
        kinematics: KinematicsInterface,
        joint_names: Tuple[str, ...],
        base_frame: str,
        tool_frame: str,
        initial_joint_positions: JointPositions,
        default_ik_policy: IKPolicy,
        seed_policy: IKSeedPolicy,
        joint_metadata: Tuple[JointMetadata, ...] = (),
    ) -> None:
        self._kinematics = kinematics
        self._joint_names = joint_names
        self._base_frame = base_frame
        self._tool_frame = tool_frame
        self._default_ik_policy = default_ik_policy
        self._seed_policy = seed_policy
        self._joint_metadata = joint_metadata
        self._last_safe_ik_seed = initial_joint_positions
        self._last_safe_tcp_pose = self._kinematics.forward(
            initial_joint_positions
        )
        self._measured_joint_positions: Optional[JointPositions] = None
        self._measured_received_at_s: Optional[float] = None
        self._pending_target: Optional[_PendingTarget] = None
        self._last_result: Optional[IKResult] = None

    @classmethod
    def from_config(
        cls,
        service_config: Mapping[str, Any],
        package_share_directory: Callable[[str], str],
        robot_config: Optional[Mapping[str, Any]] = None,
    ) -> KinematicsService:
        """Create the configured service using an injected package resolver.

        ``robot_config`` is the shared ``robot`` block in ``services.yaml``.
        It is optional only to keep direct SDK construction compatible while
        callers migrate from the former kinematics-local named-pose config.
        """
        require_enabled(service_config, "kinematics")
        if service_config.get("plugin_adapter") != "pinocchio":
            raise ValueError("Only the pinocchio kinematics plugin adapter is available")

        adapter_config = load_sdk_yaml(str(service_config["plugin_config"]))
        if adapter_config.get("plugin_adapter") != "pinocchio":
            raise ValueError("Kinematics plugin config must select pinocchio")
        frames = cls._mapping(adapter_config.get("frames"), "kinematics frames")
        joint_order = cls._mapping(
            adapter_config.get("joint_order"), "kinematics joint_order"
        )
        if joint_order.get("source") != "urdf":
            raise ValueError("kinematics joint_order.source must be 'urdf'")
        joint_convention = cls._mapping(
            adapter_config.get("joint_convention"), "kinematics joint_convention"
        )
        if (
            joint_convention.get("positive_direction")
            != "right_hand_rule_about_urdf_axis"
        ):
            raise ValueError(
                "kinematics positive direction must follow the URDF right-hand rule"
            )
        joint_names = tuple(str(name) for name in joint_order["names"])
        adapter_robot_description = cls._mapping(
            adapter_config.get("robot_description"), "robot_description"
        )
        if robot_config is None:
            robot_description = adapter_robot_description
            named_pose_config = service_config
        else:
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
                    "kinematics adapter robot_description must match the shared "
                    "robot.robot_description"
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
            named_pose_config = robot_config
        description_share = Path(
            package_share_directory(str(robot_description["package"]))
        )
        urdf_path = description_share / str(robot_description["relative_path"])
        policy = PinocchioKinematicsAdapter.policy_from_config(
            cls._mapping(adapter_config.get("solver"), "kinematics solver"),
            cls._mapping(adapter_config.get("joint_limits"), "joint_limits"),
        )
        kinematics = PinocchioKinematicsAdapter(
            urdf_path=urdf_path,
            joint_names=joint_names,
            base_frame=str(frames["base"]),
            tool_frame=str(frames["tool"]),
            default_policy=policy,
        )

        named_poses = cls._mapping(named_pose_config.get("named_poses"), "named_poses")
        initial_pose_name_value = service_config.get("initial_seed_named_pose")
        if initial_pose_name_value is None:
            # Compatibility for an older development manifest.  The value is
            # still only a seed; it is never emitted as a command.
            initial_pose_name_value = service_config.get("initial_named_pose")
        if not isinstance(initial_pose_name_value, str) or not initial_pose_name_value:
            raise ValueError("kinematics initial_seed_named_pose must be non-empty")
        initial_pose_name = initial_pose_name_value
        try:
            initial_values = named_poses[initial_pose_name]["positions_rad"]
        except KeyError as error:
            raise ValueError(
                f"Unknown initial_seed_named_pose '{initial_pose_name}'"
            ) from error
        initial_joint_positions = JointPositions(initial_values)
        if kinematics.joint_limit_violations(initial_joint_positions):
            raise ValueError("initial_seed_named_pose violates URDF joint limits")

        seed_config = cls._mapping(service_config.get("seed"), "kinematics seed")
        seed_policy = IKSeedPolicy(
            source=IKSeedSource(str(seed_config["source"])),
            measured_state_max_age_s=float(seed_config["measured_state_max_age_s"]),
            allow_last_commanded_fallback=bool(
                seed_config["allow_last_commanded_fallback"]
            ),
        )
        return cls(
            kinematics=kinematics,
            joint_names=kinematics.joint_names,
            base_frame=kinematics.base_frame,
            tool_frame=kinematics.tool_frame,
            initial_joint_positions=initial_joint_positions,
            default_ik_policy=policy,
            seed_policy=seed_policy,
            joint_metadata=kinematics.joint_metadata,
        )

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
    def joint_metadata(self) -> Tuple[JointMetadata, ...]:
        return self._joint_metadata

    @property
    def last_result(self) -> Optional[IKResult]:
        return self._last_result

    def update_measured_joint_positions(
        self, joints: JointPositions, received_at_monotonic_s: Optional[float] = None
    ) -> None:
        """Store canonical model-space feedback for future IK seeds and FK state."""
        violations = self._joint_limit_violations(joints)
        if violations:
            raise ValueError(
                "measured joint state violates URDF limits: {}".format(
                    ", ".join(violations)
                )
            )
        self._measured_joint_positions = joints
        self._measured_received_at_s = (
            time.monotonic()
            if received_at_monotonic_s is None
            else float(received_at_monotonic_s)
        )

    def set_target_pose(
        self,
        pose: Pose,
        seed: Optional[JointPositions] = None,
        policy: Optional[IKPolicy] = None,
    ) -> None:
        """Queue a target pose; optional seed makes this an explicit IK request."""
        self._pending_target = _PendingTarget(
            target_pose=pose,
            policy=policy or self._default_ik_policy,
            explicit_seed=seed,
        )

    def request_ik(self, request: IKRequest) -> None:
        """Queue the complete SDK input ``target pose + seed + policy``."""
        self._pending_target = _PendingTarget(
            target_pose=request.target_pose,
            policy=request.policy,
            explicit_seed=request.seed,
        )

    def clear_target_pose(self) -> None:
        """Discard a pending target without altering measured or commanded state."""
        self._pending_target = None

    def step(self, now_monotonic_s: Optional[float] = None) -> KinematicsStep:
        """Process at most one request and never replace the last safe command on failure."""
        now_s = time.monotonic() if now_monotonic_s is None else float(now_monotonic_s)
        measured_age_s = self._measured_state_age_s(now_s)
        measured_fresh = (
            measured_age_s is not None
            and measured_age_s <= self._seed_policy.measured_state_max_age_s
        )
        measured_pose = (
            self._kinematics.forward(self._measured_joint_positions)
            if self._measured_joint_positions is not None
            else None
        )

        target_processed = self._pending_target is not None
        command_updated = False
        joint_goal: Optional[JointPositions] = None
        result: Optional[IKResult] = None
        seed_source: Optional[IKSeedSource] = None
        if self._pending_target is not None:
            pending = self._pending_target
            self._pending_target = None
            seed, seed_source = self._resolve_seed(pending, measured_fresh)
            if seed is None:
                result = self._seed_unavailable_result()
            else:
                result = self._kinematics.solve_ik(
                    IKRequest(
                        target_pose=pending.target_pose,
                        seed=seed,
                        policy=pending.policy,
                    )
                )
                if result.converged and result.q_solution is not None:
                    self._last_safe_ik_seed = result.q_solution
                    self._last_safe_tcp_pose = self._kinematics.forward(
                        result.q_solution
                    )
                    command_updated = True
                    joint_goal = result.q_solution
            self._last_result = result

        return KinematicsStep(
            commanded_joint_positions=self._last_safe_ik_seed,
            commanded_tcp_pose=self._last_safe_tcp_pose,
            joint_goal=joint_goal,
            measured_joint_positions=self._measured_joint_positions,
            measured_tcp_pose=measured_pose,
            measured_state_age_s=measured_age_s,
            measured_state_fresh=measured_fresh,
            target_processed=target_processed,
            command_updated=command_updated,
            ik_result=result,
            seed_source=seed_source,
        )

    def _resolve_seed(
        self, pending: _PendingTarget, measured_fresh: bool
    ) -> Tuple[Optional[JointPositions], Optional[IKSeedSource]]:
        if pending.explicit_seed is not None:
            return pending.explicit_seed, IKSeedSource.EXPLICIT
        if self._seed_policy.source is IKSeedSource.LAST_COMMANDED:
            return self._last_safe_ik_seed, IKSeedSource.LAST_COMMANDED
        if (
            self._seed_policy.source is IKSeedSource.MEASURED_JOINT_STATE
            and self._measured_joint_positions is not None
            and measured_fresh
        ):
            return self._measured_joint_positions, IKSeedSource.MEASURED_JOINT_STATE
        if self._seed_policy.allow_last_commanded_fallback:
            return self._last_safe_ik_seed, IKSeedSource.LAST_COMMANDED
        return None, self._seed_policy.source

    def _seed_unavailable_result(self) -> IKResult:
        return IKResult(
            q_solution=None,
            converged=False,
            failure_reason=IKFailureReason.SEED_UNAVAILABLE,
            detail=(
                "fresh canonical measured_joint_state is required by the configured "
                "seed policy"
            ),
            position_residual_m=float("nan"),
            orientation_residual_rad=float("nan"),
            iteration_count=0,
            singularity=SingularityMetrics(
                minimum_singular_value=float("nan"),
                condition_number=float("nan"),
                rank=0,
                near_singular=False,
                singular=False,
            ),
            seed=None,
            active_joint_limits=(),
            minimum_joint_limit_margin_rad=float("nan"),
        )

    def _joint_limit_violations(self, joints: JointPositions) -> Tuple[str, ...]:
        validator = getattr(self._kinematics, "joint_limit_violations", None)
        if validator is None:
            return ()
        return tuple(validator(joints))

    def _measured_state_age_s(self, now_s: float) -> Optional[float]:
        if self._measured_received_at_s is None:
            return None
        return max(0.0, now_s - self._measured_received_at_s)

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value
