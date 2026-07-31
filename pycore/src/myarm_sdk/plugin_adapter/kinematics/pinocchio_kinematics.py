"""Pinocchio FK/IK adapter with URDF-backed safety diagnostics."""

from __future__ import annotations

import math
import time
import xml.etree.ElementTree as element_tree
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple

import numpy as np

from myarm_sdk.core import (
    IKFailureReason,
    IKNearSingularityPolicy,
    IKPolicy,
    IKRequest,
    IKResult,
    IKTaskMode,
    JointMetadata,
    JointPositions,
    Pose,
    SingularityMetrics,
)
from myarm_sdk.core.spatial import (
    quaternion_xyzw_from_rotation_matrix,
    rotation_matrix_from_quaternion_xyzw,
)


class InverseKinematicsError(RuntimeError):
    """Compatibility error raised only by the legacy :meth:`inverse` wrapper."""


class PinocchioKinematicsAdapter:
    """Compute base-to-tool FK and bounded, diagnostic damped-least-squares IK.

    Joint axes, hard limits and fixed TCP placement are read from the selected
    URDF.  YAML provides an expected order and safety/solver policy, but does
    not duplicate kinematic facts that Pinocchio already owns.
    """

    DEFAULT_JOINT_NAMES = (
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
        joint_names: Sequence[str] = DEFAULT_JOINT_NAMES,
        base_frame: str = "base_link",
        tool_frame: str = "tool0",
        default_policy: Optional[IKPolicy] = None,
    ) -> None:
        if not urdf_path.is_file():
            raise ValueError(f"Pinocchio URDF file does not exist: {urdf_path}")

        self._joint_names = tuple(str(name) for name in joint_names)
        if len(self._joint_names) != 6 or len(set(self._joint_names)) != 6:
            raise ValueError("MyArm M750 joint_order must contain six unique joint names")
        self._base_frame = str(base_frame)
        self._tool_frame = str(tool_frame)
        self._default_policy = default_policy or IKPolicy()

        try:
            import pinocchio as pin
        except ImportError as error:
            raise RuntimeError(
                "Install kinematics support with `pip install myarm-sdk[kinematics]`."
            ) from error

        self._pin = pin
        urdf_root = element_tree.parse(str(urdf_path)).getroot()
        self._joint_metadata = self._read_joint_metadata(urdf_root, self._joint_names)
        self._conservative_max_reach_m = self._read_conservative_max_reach(
            urdf_root, self._base_frame, self._tool_frame
        )

        full_model = pin.buildModelFromUrdf(str(urdf_path))
        unknown_joints = [
            name for name in self._joint_names if not full_model.existJointName(name)
        ]
        if unknown_joints:
            raise ValueError("URDF is missing arm joints: {}".format(", ".join(unknown_joints)))

        locked_joint_ids = [
            joint_id
            for joint_id, name in enumerate(full_model.names)
            if joint_id != 0 and name not in self._joint_names
        ]
        self._model = pin.buildReducedModel(
            full_model, locked_joint_ids, pin.neutral(full_model)
        )
        if self._model.nq != len(self._joint_names):
            raise ValueError(
                f"reduced Pinocchio model must have six joint coordinates, got {self._model.nq}"
            )
        self._validate_joint_order_and_limits()

        frame_names = [frame.name for frame in self._model.frames]
        if self._base_frame not in frame_names:
            raise ValueError(f"URDF is missing base frame: {self._base_frame}")
        if self._tool_frame not in frame_names:
            raise ValueError(f"URDF is missing tool frame: {self._tool_frame}")
        self._base_frame_id = self._model.getFrameId(self._base_frame)
        self._tool_frame_id = self._model.getFrameId(self._tool_frame)
        if int(self._model.frames[self._base_frame_id].parent) != 0:
            raise ValueError(
                "base_frame must be fixed to the URDF root for this kinematics adapter"
            )

        self._lower_limits = np.asarray(self._model.lowerPositionLimit, dtype=float)
        self._upper_limits = np.asarray(self._model.upperPositionLimit, dtype=float)
        self._data = self._model.createData()

    @classmethod
    def policy_from_config(
        cls,
        solver_config: Mapping[str, Any],
        joint_limit_config: Mapping[str, Any],
    ) -> IKPolicy:
        """Build a typed IK policy from the selected Pinocchio YAML config."""
        if joint_limit_config.get("source") != "urdf":
            raise ValueError("Pinocchio joint_limits.source must be 'urdf'")
        task_config = cls._mapping(solver_config.get("task"), "solver.task")
        singularity_config = cls._mapping(
            solver_config.get("singularity"), "solver.singularity"
        )
        return IKPolicy(
            task_mode=IKTaskMode(str(task_config["default_mode"])),
            position_mask=cls._mask(task_config["position_mask"], "position_mask"),
            orientation_mask=cls._mask(
                task_config["orientation_mask"], "orientation_mask"
            ),
            position_tolerance_m=float(solver_config["position_tolerance_m"]),
            orientation_tolerance_rad=float(solver_config["orientation_tolerance_rad"]),
            damping=float(solver_config["damping"]),
            step_size=float(solver_config["step_size"]),
            max_joint_step_rad=float(solver_config["max_joint_step_rad"]),
            max_iterations=int(solver_config["max_iterations"]),
            max_solve_time_ms=float(solver_config["max_solve_time_ms"]),
            characteristic_length_m=float(solver_config["characteristic_length_m"]),
            safety_limit_margin_rad=float(joint_limit_config["safety_margin_rad"]),
            seed_outside_tolerance_rad=float(
                joint_limit_config["seed_outside_tolerance_rad"]
            ),
            near_minimum_singular_value=float(
                singularity_config["near_minimum_singular_value"]
            ),
            singular_minimum_singular_value=float(
                singularity_config["singular_minimum_singular_value"]
            ),
            near_condition_number=float(singularity_config["near_condition_number"]),
            singular_condition_number=float(
                singularity_config["singular_condition_number"]
            ),
            near_singularity_policy=IKNearSingularityPolicy(
                str(singularity_config["policy"])
            ),
        )

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """Canonical order verified against the reduced Pinocchio model."""
        return self._joint_names

    @property
    def joint_metadata(self) -> Tuple[JointMetadata, ...]:
        """URDF axis, hard-limit and velocity metadata in canonical order."""
        return self._joint_metadata

    @property
    def base_frame(self) -> str:
        return self._base_frame

    @property
    def tool_frame(self) -> str:
        return self._tool_frame

    @property
    def default_policy(self) -> IKPolicy:
        return self._default_policy

    def forward(self, joints: JointPositions) -> Pose:
        """Return the configured ``base_frame -> tool_frame`` FK pose."""
        configuration = self._as_configuration(joints.values)
        placement = self._base_to_tool_placement(configuration)
        return Pose(
            position=tuple(float(value) for value in placement.translation),
            orientation=quaternion_xyzw_from_rotation_matrix(placement.rotation),
        )

    def solve_ik(self, request: IKRequest) -> IKResult:
        """Solve one bounded IK request and return diagnostics instead of throwing.

        The solver uses quaternions/SE(3) throughout.  It validates hard limits
        before solving, projects each candidate to the configured safe range,
        and only returns ``q_solution`` after a post-solve hard-limit check.
        """
        started_at_s = time.monotonic()
        policy = request.policy
        try:
            configuration = self._as_configuration(request.seed.values)
            desired_base_to_tool = self._pin.SE3(
                rotation_matrix_from_quaternion_xyzw(request.target_pose.orientation),
                np.asarray(request.target_pose.position, dtype=float),
            )
        except (ValueError, TypeError) as error:
            return self._failure(
                reason=IKFailureReason.INVALID_TARGET,
                detail=str(error),
                seed=request.seed,
            )

        hard_limit_violations = self.joint_limit_violations(
            request.seed, tolerance_rad=policy.seed_outside_tolerance_rad
        )
        if hard_limit_violations:
            return self._failure(
                reason=IKFailureReason.SEED_OUT_OF_LIMIT,
                detail="IK seed violates URDF hard joint limits",
                seed=request.seed,
                active_joint_limits=hard_limit_violations,
                minimum_joint_limit_margin_rad=self._minimum_hard_limit_margin(
                    configuration
                ),
            )

        evaluation = self._evaluate(configuration, desired_base_to_tool, policy)
        if (
            float(np.linalg.norm(desired_base_to_tool.translation))
            > self._conservative_max_reach_m + policy.position_tolerance_m
        ):
            return self._failure_from_evaluation(
                reason=IKFailureReason.UNREACHABLE,
                detail=(
                    "target lies outside the conservative URDF base-to-tool reach "
                    "bound"
                ),
                seed=request.seed,
                evaluation=evaluation,
            )

        safe_lower, safe_upper = self._safe_joint_limits(policy)
        active_joint_limits = set()
        for iteration_count in range(policy.max_iterations + 1):
            if iteration_count > 0 and self._timed_out(started_at_s, policy):
                return self._failure_from_evaluation(
                    reason=IKFailureReason.TIMEOUT,
                    detail="IK exceeded max_solve_time_ms",
                    seed=request.seed,
                    evaluation=evaluation,
                    iteration_count=iteration_count,
                    active_joint_limits=tuple(sorted(active_joint_limits)),
                )

            evaluation = self._evaluate(configuration, desired_base_to_tool, policy)
            if self._converged(evaluation, policy):
                return self._accepted_or_singularity_failure(
                    configuration=configuration,
                    seed=request.seed,
                    evaluation=evaluation,
                    policy=policy,
                    iteration_count=iteration_count,
                    active_joint_limits=tuple(sorted(active_joint_limits)),
                )

            if iteration_count == policy.max_iterations:
                break

            try:
                delta = self._damped_least_squares_delta(evaluation, policy)
            except np.linalg.LinAlgError:
                return self._failure_from_evaluation(
                    reason=IKFailureReason.SINGULAR,
                    detail="IK task Jacobian could not be solved numerically",
                    seed=request.seed,
                    evaluation=evaluation,
                    iteration_count=iteration_count,
                    active_joint_limits=tuple(sorted(active_joint_limits)),
                )

            candidate_unbounded = self._pin.integrate(self._model, configuration, delta)
            candidate = np.minimum(
                np.maximum(candidate_unbounded, safe_lower), safe_upper
            )
            newly_active = self._projected_joint_names(
                candidate_unbounded, candidate, safe_lower, safe_upper
            )
            active_joint_limits.update(newly_active)
            if not np.isfinite(candidate).all():
                return self._failure_from_evaluation(
                    reason=IKFailureReason.SINGULAR,
                    detail="IK produced a non-finite joint update",
                    seed=request.seed,
                    evaluation=evaluation,
                    iteration_count=iteration_count,
                    active_joint_limits=tuple(sorted(active_joint_limits)),
                )
            if np.allclose(candidate, configuration, rtol=0.0, atol=1e-12):
                reason = (
                    IKFailureReason.JOINT_LIMIT_BLOCKED
                    if newly_active
                    else IKFailureReason.SINGULAR
                    if evaluation.singularity.singular
                    else IKFailureReason.MAX_ITERATIONS
                )
                return self._failure_from_evaluation(
                    reason=reason,
                    detail="IK update was fully blocked before convergence",
                    seed=request.seed,
                    evaluation=evaluation,
                    iteration_count=iteration_count,
                    active_joint_limits=tuple(sorted(active_joint_limits)),
                )
            configuration = candidate

        evaluation = self._evaluate(configuration, desired_base_to_tool, policy)
        if active_joint_limits:
            reason = IKFailureReason.JOINT_LIMIT_BLOCKED
            detail = "IK reached configured software joint-limit margins"
        elif evaluation.singularity.singular:
            reason = IKFailureReason.SINGULAR
            detail = "IK remained rank-deficient near a singularity"
        elif evaluation.position_residual_m > policy.position_tolerance_m * 10.0:
            reason = IKFailureReason.UNREACHABLE
            detail = "IK could not reduce the Cartesian position residual"
        else:
            reason = IKFailureReason.MAX_ITERATIONS
            detail = "IK reached max_iterations without satisfying tolerances"
        return self._failure_from_evaluation(
            reason=reason,
            detail=detail,
            seed=request.seed,
            evaluation=evaluation,
            iteration_count=policy.max_iterations,
            active_joint_limits=tuple(sorted(active_joint_limits)),
        )

    def inverse(self, pose: Pose, seed: JointPositions) -> JointPositions:
        """Legacy convenience wrapper; new callers should use :meth:`solve_ik`."""
        result = self.solve_ik(IKRequest(pose, seed, self._default_policy))
        if not result.converged or result.q_solution is None:
            reason = result.failure_reason.value if result.failure_reason else "unknown"
            raise InverseKinematicsError(f"{reason}: {result.detail}")
        return result.q_solution

    def joint_limit_violations(
        self, joints: JointPositions, tolerance_rad: float = 0.0
    ) -> Tuple[str, ...]:
        """Return canonical names outside their URDF hard bounds."""
        configuration = self._as_configuration(joints.values)
        lower = self._lower_limits - tolerance_rad
        upper = self._upper_limits + tolerance_rad
        return tuple(
            name
            for name, value, lower_value, upper_value in zip(
                self._joint_names, configuration, lower, upper
            )
            if value < lower_value or value > upper_value
        )

    def _accepted_or_singularity_failure(
        self,
        configuration: np.ndarray,
        seed: JointPositions,
        evaluation: _IKEvaluation,
        policy: IKPolicy,
        iteration_count: int,
        active_joint_limits: Tuple[str, ...],
    ) -> IKResult:
        hard_violations = self._hard_limit_violations_from_configuration(configuration)
        if hard_violations:
            return self._failure_from_evaluation(
                reason=IKFailureReason.JOINT_LIMIT_BLOCKED,
                detail="post-solve joint-limit validation failed",
                seed=seed,
                evaluation=evaluation,
                iteration_count=iteration_count,
                active_joint_limits=hard_violations,
            )
        if self._outside_safe_limits(configuration, policy):
            return self._failure_from_evaluation(
                reason=IKFailureReason.JOINT_LIMIT_BLOCKED,
                detail="solution violates configured software joint-limit margin",
                seed=seed,
                evaluation=evaluation,
                iteration_count=iteration_count,
                active_joint_limits=active_joint_limits
                or self._active_safe_limit_names(configuration, policy),
            )
        if (
            policy.near_singularity_policy is IKNearSingularityPolicy.REJECT
            and evaluation.singularity.near_singular
        ):
            reason = (
                IKFailureReason.SINGULAR
                if evaluation.singularity.singular
                else IKFailureReason.NEAR_SINGULAR
            )
            return self._failure_from_evaluation(
                reason=reason,
                detail="converged solution is rejected by singularity policy",
                seed=seed,
                evaluation=evaluation,
                iteration_count=iteration_count,
                active_joint_limits=active_joint_limits,
            )
        return IKResult(
            q_solution=JointPositions(configuration),
            converged=True,
            failure_reason=None,
            detail=(
                "IK converged near a singularity"
                if evaluation.singularity.near_singular
                else "IK converged"
            ),
            position_residual_m=evaluation.position_residual_m,
            orientation_residual_rad=evaluation.orientation_residual_rad,
            iteration_count=iteration_count,
            singularity=evaluation.singularity,
            seed=seed,
            active_joint_limits=active_joint_limits,
            minimum_joint_limit_margin_rad=self._minimum_hard_limit_margin(
                configuration
            ),
        )

    def _failure(
        self,
        reason: IKFailureReason,
        detail: str,
        seed: Optional[JointPositions],
        active_joint_limits: Tuple[str, ...] = (),
        minimum_joint_limit_margin_rad: float = float("nan"),
    ) -> IKResult:
        return IKResult(
            q_solution=None,
            converged=False,
            failure_reason=reason,
            detail=detail,
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
            seed=seed,
            active_joint_limits=active_joint_limits,
            minimum_joint_limit_margin_rad=minimum_joint_limit_margin_rad,
        )

    def _failure_from_evaluation(
        self,
        reason: IKFailureReason,
        detail: str,
        seed: JointPositions,
        evaluation: _IKEvaluation,
        iteration_count: int = 0,
        active_joint_limits: Tuple[str, ...] = (),
    ) -> IKResult:
        return IKResult(
            q_solution=None,
            converged=False,
            failure_reason=reason,
            detail=detail,
            position_residual_m=evaluation.position_residual_m,
            orientation_residual_rad=evaluation.orientation_residual_rad,
            iteration_count=iteration_count,
            singularity=evaluation.singularity,
            seed=seed,
            active_joint_limits=active_joint_limits,
            minimum_joint_limit_margin_rad=self._minimum_hard_limit_margin(
                evaluation.configuration
            ),
        )

    def _evaluate(self, configuration: np.ndarray, desired, policy: IKPolicy) -> _IKEvaluation:
        current = self._base_to_tool_placement(configuration)
        error_transform = current.actInv(desired)
        error = np.asarray(self._pin.log6(error_transform).vector, dtype=float)
        position_residual_m = float(np.linalg.norm(error[:3]))
        orientation_residual_rad = float(np.linalg.norm(error[3:]))

        frame_jacobian = self._pin.computeFrameJacobian(
            self._model,
            self._data,
            configuration,
            self._tool_frame_id,
            self._pin.ReferenceFrame.LOCAL,
        )
        task_jacobian = self._pin.Jlog6(error_transform.inverse()) @ frame_jacobian
        active_indices = np.flatnonzero(np.asarray(policy.active_mask, dtype=bool))
        active_error = error[active_indices]
        active_jacobian = task_jacobian[active_indices, :]
        weights = np.asarray(
            [1.0 / policy.characteristic_length_m if index < 3 else 1.0 for index in active_indices],
            dtype=float,
        )
        weighted_error = weights * active_error
        weighted_jacobian = weights[:, np.newaxis] * active_jacobian
        return _IKEvaluation(
            configuration=configuration,
            error=error,
            weighted_error=weighted_error,
            weighted_jacobian=weighted_jacobian,
            position_residual_m=position_residual_m,
            orientation_residual_rad=orientation_residual_rad,
            singularity=self._singularity_metrics(weighted_jacobian, policy),
        )

    def _damped_least_squares_delta(
        self, evaluation: _IKEvaluation, policy: IKPolicy
    ) -> np.ndarray:
        jacobian = evaluation.weighted_jacobian
        system = jacobian @ jacobian.T + policy.damping * np.eye(jacobian.shape[0])
        velocity = jacobian.T @ np.linalg.solve(system, evaluation.weighted_error)
        return np.clip(
            policy.step_size * velocity,
            -policy.max_joint_step_rad,
            policy.max_joint_step_rad,
        )

    def _base_to_tool_placement(self, configuration: np.ndarray):
        self._pin.forwardKinematics(self._model, self._data, configuration)
        self._pin.updateFramePlacements(self._model, self._data)
        base = self._data.oMf[self._base_frame_id]
        tool = self._data.oMf[self._tool_frame_id]
        return base.actInv(tool)

    def _converged(self, evaluation: _IKEvaluation, policy: IKPolicy) -> bool:
        position_mask = np.asarray(policy.position_mask, dtype=bool)
        orientation_mask = np.asarray(policy.active_mask[3:], dtype=bool)
        position_ok = (
            not position_mask.any()
            or float(np.linalg.norm(evaluation.error[:3][position_mask]))
            <= policy.position_tolerance_m
        )
        orientation_ok = (
            not orientation_mask.any()
            or float(np.linalg.norm(evaluation.error[3:][orientation_mask]))
            <= policy.orientation_tolerance_rad
        )
        return bool(position_ok and orientation_ok)

    def _safe_joint_limits(self, policy: IKPolicy) -> Tuple[np.ndarray, np.ndarray]:
        lower = self._lower_limits + policy.safety_limit_margin_rad
        upper = self._upper_limits - policy.safety_limit_margin_rad
        if np.any(lower >= upper):
            raise ValueError("joint_limit safety_margin_rad leaves no valid range")
        return lower, upper

    def _outside_safe_limits(self, configuration: np.ndarray, policy: IKPolicy) -> bool:
        lower, upper = self._safe_joint_limits(policy)
        return bool(np.any(configuration < lower - 1e-10) or np.any(configuration > upper + 1e-10))

    def _active_safe_limit_names(
        self, configuration: np.ndarray, policy: IKPolicy
    ) -> Tuple[str, ...]:
        lower, upper = self._safe_joint_limits(policy)
        return tuple(
            name
            for name, value, lower_value, upper_value in zip(
                self._joint_names, configuration, lower, upper
            )
            if math.isclose(value, lower_value, abs_tol=1e-8)
            or math.isclose(value, upper_value, abs_tol=1e-8)
        )

    def _projected_joint_names(
        self,
        unbounded: np.ndarray,
        bounded: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
    ) -> Tuple[str, ...]:
        return tuple(
            name
            for name, before, after, lower_value, upper_value in zip(
                self._joint_names, unbounded, bounded, lower, upper
            )
            if not math.isclose(before, after, rel_tol=0.0, abs_tol=1e-12)
            or math.isclose(after, lower_value, abs_tol=1e-8)
            or math.isclose(after, upper_value, abs_tol=1e-8)
        )

    def _hard_limit_violations_from_configuration(
        self, configuration: np.ndarray
    ) -> Tuple[str, ...]:
        return tuple(
            name
            for name, value, lower, upper in zip(
                self._joint_names, configuration, self._lower_limits, self._upper_limits
            )
            if value < lower - 1e-10 or value > upper + 1e-10
        )

    def _minimum_hard_limit_margin(self, configuration: np.ndarray) -> float:
        return float(
            min(
                np.min(configuration - self._lower_limits),
                np.min(self._upper_limits - configuration),
            )
        )

    @staticmethod
    def _timed_out(started_at_s: float, policy: IKPolicy) -> bool:
        return (time.monotonic() - started_at_s) * 1000.0 >= policy.max_solve_time_ms

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    @staticmethod
    def _mask(value: Any, name: str) -> Tuple[bool, bool, bool]:
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError(f"{name} must contain exactly three boolean values")
        return tuple(bool(item) for item in value)  # type: ignore[return-value]

    @staticmethod
    def _axis(value: str, name: str) -> Tuple[float, float, float]:
        axis = tuple(float(item) for item in value.split())
        if len(axis) != 3:
            raise ValueError(f"URDF joint {name} axis must have three values")
        norm = math.sqrt(sum(item * item for item in axis))
        if norm < 1e-12:
            raise ValueError(f"URDF joint {name} axis must not be zero")
        return tuple(item / norm for item in axis)  # type: ignore[return-value]

    @classmethod
    def _read_joint_metadata(
        cls, root, joint_names: Sequence[str]
    ) -> Tuple[JointMetadata, ...]:
        joint_elements = {
            str(element.attrib["name"]): element for element in root.findall("joint")
        }
        metadata = []
        for name in joint_names:
            element = joint_elements.get(name)
            if element is None:
                raise ValueError(f"URDF is missing joint metadata for {name}")
            if element.attrib.get("type") != "revolute":
                raise ValueError(f"URDF arm joint {name} must be revolute")
            axis_element = element.find("axis")
            limit_element = element.find("limit")
            if axis_element is None or limit_element is None:
                raise ValueError(f"URDF arm joint {name} requires axis and limit")
            metadata.append(
                JointMetadata(
                    name=name,
                    axis_xyz=cls._axis(str(axis_element.attrib.get("xyz", "1 0 0")), name),
                    lower_limit_rad=float(limit_element.attrib["lower"]),
                    upper_limit_rad=float(limit_element.attrib["upper"]),
                    velocity_limit_rad_s=float(limit_element.attrib["velocity"]),
                )
            )
        return tuple(metadata)

    @staticmethod
    def _read_conservative_max_reach(root, base_frame: str, tool_frame: str) -> float:
        child_to_joint = {}
        for element in root.findall("joint"):
            child = element.find("child")
            parent = element.find("parent")
            if child is not None and parent is not None:
                child_to_joint[str(child.attrib["link"])] = element

        total = 0.0
        current = tool_frame
        while current != base_frame:
            element = child_to_joint.get(current)
            if element is None:
                raise ValueError(
                    f"URDF has no parent-joint chain from {tool_frame} to {base_frame}"
                )
            origin = element.find("origin")
            if origin is not None:
                xyz = tuple(float(item) for item in origin.attrib.get("xyz", "0 0 0").split())
                if len(xyz) != 3:
                    raise ValueError("URDF joint origin xyz must have three values")
                total += math.sqrt(sum(item * item for item in xyz))
            parent = element.find("parent")
            current = str(parent.attrib["link"])
        return total

    def _validate_joint_order_and_limits(self) -> None:
        model_joint_names = tuple(self._model.names[index] for index in range(1, self._model.njoints))
        if model_joint_names != self._joint_names:
            raise ValueError(
                "joint_order.names must match the canonical URDF/Pinocchio order; "
                f"URDF order is {model_joint_names}"
            )
        lower = np.asarray(self._model.lowerPositionLimit, dtype=float)
        upper = np.asarray(self._model.upperPositionLimit, dtype=float)
        for metadata, lower_limit, upper_limit in zip(
            self._joint_metadata, lower, upper
        ):
            if not (
                math.isclose(metadata.lower_limit_rad, float(lower_limit), abs_tol=1e-9)
                and math.isclose(metadata.upper_limit_rad, float(upper_limit), abs_tol=1e-9)
            ):
                raise ValueError(
                    f"Pinocchio limits for {metadata.name} disagree with URDF metadata"
                )

    def _as_configuration(self, values: Sequence[float]) -> np.ndarray:
        configuration = np.asarray(values, dtype=float)
        if configuration.shape != (len(self._joint_names),):
            raise ValueError("MyArm M750 requires exactly six joint values")
        if not np.isfinite(configuration).all():
            raise ValueError("joint values must be finite")
        return configuration

    @staticmethod
    def _singularity_metrics(
        weighted_jacobian: np.ndarray, policy: IKPolicy
    ) -> SingularityMetrics:
        singular_values = np.linalg.svd(weighted_jacobian, compute_uv=False)
        minimum = float(singular_values[-1])
        maximum = float(singular_values[0])
        condition = float("inf") if minimum <= 1e-15 else maximum / minimum
        rank = int(
            np.sum(singular_values > policy.singular_minimum_singular_value)
        )
        near = (
            minimum <= policy.near_minimum_singular_value
            or condition >= policy.near_condition_number
        )
        singular = (
            rank < min(weighted_jacobian.shape)
            or minimum <= policy.singular_minimum_singular_value
            or condition >= policy.singular_condition_number
        )
        return SingularityMetrics(
            minimum_singular_value=minimum,
            condition_number=condition,
            rank=rank,
            near_singular=near,
            singular=singular,
        )


class _IKEvaluation:
    """Private per-iteration values kept out of the public SDK result contract."""

    def __init__(
        self,
        configuration: np.ndarray,
        error: np.ndarray,
        weighted_error: np.ndarray,
        weighted_jacobian: np.ndarray,
        position_residual_m: float,
        orientation_residual_rad: float,
        singularity: SingularityMetrics,
    ) -> None:
        self.configuration = configuration
        self.error = error
        self.weighted_error = weighted_error
        self.weighted_jacobian = weighted_jacobian
        self.position_residual_m = position_residual_m
        self.orientation_residual_rad = orientation_residual_rad
        self.singularity = singularity
