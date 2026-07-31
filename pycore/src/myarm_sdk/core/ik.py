"""ROS-independent request, policy and result types for inverse kinematics."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple

from .joint_positions import JointPositions
from .pose import Pose


class IKTaskMode(str, Enum):
    """The Cartesian constraints that an IK solve must satisfy."""

    FULL_POSE = "full_pose"
    POSITION_ONLY = "position_only"


class IKNearSingularityPolicy(str, Enum):
    """How a converged solution near a singularity is handled."""

    WARN = "warn"
    REJECT = "reject"


class IKSeedSource(str, Enum):
    """Where a service obtains the seed for a solve."""

    MEASURED_JOINT_STATE = "measured_joint_state"
    LAST_COMMANDED = "last_commanded"
    EXPLICIT = "explicit"


class IKFailureReason(str, Enum):
    """A safe, machine-readable reason why no new command was accepted."""

    INVALID_TARGET = "invalid_target"
    SEED_UNAVAILABLE = "seed_unavailable"
    SEED_OUT_OF_LIMIT = "seed_out_of_limit"
    UNREACHABLE = "unreachable"
    JOINT_LIMIT_BLOCKED = "joint_limit_blocked"
    SINGULAR = "singular"
    NEAR_SINGULAR = "near_singular"
    TIMEOUT = "timeout"
    MAX_ITERATIONS = "max_iterations"


@dataclass(frozen=True)
class IKPolicy:
    """Numerical and safety policy for one IK request.

    All orientation constraints operate on the rotational tangent part of an
    SE(3) logarithm, never on Euler angles.
    """

    task_mode: IKTaskMode = IKTaskMode.FULL_POSE
    position_mask: Tuple[bool, bool, bool] = (True, True, True)
    orientation_mask: Tuple[bool, bool, bool] = (True, True, True)
    position_tolerance_m: float = 0.001
    orientation_tolerance_rad: float = 0.02
    damping: float = 0.001
    step_size: float = 0.5
    max_joint_step_rad: float = 0.10
    max_iterations: int = 150
    max_solve_time_ms: float = 100.0
    characteristic_length_m: float = 0.20
    safety_limit_margin_rad: float = 0.02
    seed_outside_tolerance_rad: float = 0.002
    near_minimum_singular_value: float = 0.001
    singular_minimum_singular_value: float = 0.000001
    near_condition_number: float = 1000.0
    singular_condition_number: float = 1000000.0
    near_singularity_policy: IKNearSingularityPolicy = (
        IKNearSingularityPolicy.REJECT
    )

    def __post_init__(self) -> None:
        if len(self.position_mask) != 3 or len(self.orientation_mask) != 3:
            raise ValueError("IK task masks must each contain exactly three values")
        if not any(self.active_mask):
            raise ValueError("IK policy must constrain at least one task component")
        if self.position_tolerance_m <= 0.0 or self.orientation_tolerance_rad <= 0.0:
            raise ValueError("IK tolerances must be positive")
        if self.damping <= 0.0:
            raise ValueError("IK damping must be positive")
        if not 0.0 < self.step_size <= 1.0:
            raise ValueError("IK step_size must be in (0, 1]")
        if self.max_joint_step_rad <= 0.0 or self.max_iterations < 1:
            raise ValueError("IK max_joint_step_rad and max_iterations must be positive")
        if self.max_solve_time_ms <= 0.0 or self.characteristic_length_m <= 0.0:
            raise ValueError("IK time limit and characteristic length must be positive")
        if self.safety_limit_margin_rad < 0.0 or self.seed_outside_tolerance_rad < 0.0:
            raise ValueError("IK joint-limit margins must not be negative")
        if self.singular_minimum_singular_value > self.near_minimum_singular_value:
            raise ValueError("singular sigma threshold must not exceed near-singular threshold")
        if self.singular_condition_number < self.near_condition_number:
            raise ValueError("singular condition threshold must not be below near threshold")

    @property
    def active_mask(self) -> Tuple[bool, bool, bool, bool, bool, bool]:
        """Return the six SE(3) tangent components constrained by this policy."""
        orientation_mask = (
            (False, False, False)
            if self.task_mode is IKTaskMode.POSITION_ONLY
            else self.orientation_mask
        )
        return self.position_mask + orientation_mask


@dataclass(frozen=True)
class IKSeedPolicy:
    """How a kinematics service chooses an IK seed when a command arrives."""

    source: IKSeedSource = IKSeedSource.MEASURED_JOINT_STATE
    measured_state_max_age_s: float = 0.5
    allow_last_commanded_fallback: bool = False

    def __post_init__(self) -> None:
        if self.measured_state_max_age_s <= 0.0:
            raise ValueError("measured_state_max_age_s must be positive")


@dataclass(frozen=True)
class IKRequest:
    """Complete SDK input for a single inverse-kinematics solve."""

    target_pose: Pose
    seed: JointPositions
    policy: IKPolicy


@dataclass(frozen=True)
class SingularityMetrics:
    """SVD-based conditioning information for the active task Jacobian."""

    minimum_singular_value: float
    condition_number: float
    rank: int
    near_singular: bool
    singular: bool


@dataclass(frozen=True)
class IKResult:
    """The safe outcome of an IK request; failures never expose a command q."""

    q_solution: Optional[JointPositions]
    converged: bool
    failure_reason: Optional[IKFailureReason]
    detail: str
    position_residual_m: float
    orientation_residual_rad: float
    iteration_count: int
    singularity: SingularityMetrics
    seed: Optional[JointPositions]
    active_joint_limits: Tuple[str, ...]
    minimum_joint_limit_margin_rad: float

