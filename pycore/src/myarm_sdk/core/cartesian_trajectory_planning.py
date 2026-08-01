"""ROS-independent value types for Cartesian-to-joint trajectory planning.

The Cartesian planner is deliberately separate from the joint point-to-point
planner: its input is a TCP target and an explicit fresh joint seed, while its
output remains the same validated :class:`JointTrajectory` consumed by motion
execution.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Sequence, Tuple

import numpy as np

from .ik import IKPolicy
from .joint_positions import JointPositions
from .joint_trajectory import JointTrajectory
from .joint_trajectory_interpolation import minimum_jerk_position_scale
from .joint_trajectory_planning import JointMotionLimits, TimeScalingPolicy
from .pose import Pose


class CartesianPathMode(str, Enum):
    """Geometric path family before sequential inverse kinematics."""

    LINEAR_TRANSLATION_SLERP = "linear_translation_slerp"
    SE3_GEODESIC = "se3_geodesic"


class CartesianTrajectoryPlanningFailureReason(str, Enum):
    """Machine-readable all-or-nothing Cartesian planning failures."""

    START_OUT_OF_LIMIT = "start_out_of_limit"
    FK_FAILED = "fk_failed"
    INVALID_PATH = "invalid_path"
    WAYPOINT_LIMIT_EXCEEDED = "waypoint_limit_exceeded"
    UNREACHABLE = "unreachable"
    JOINT_LIMIT_BLOCKED = "joint_limit_blocked"
    SINGULAR = "singular"
    NEAR_SINGULAR = "near_singular"
    IK_TIMEOUT = "ik_timeout"
    IK_MAX_ITERATIONS = "ik_max_iterations"
    IK_FAILED = "ik_failed"
    BRANCH_DISCONTINUITY = "branch_discontinuity"
    DURATION_BELOW_LIMIT = "duration_below_limit"
    DENSE_VALIDATION_FAILED = "dense_validation_failed"
    CARTESIAN_VALIDATION_FAILED = "cartesian_validation_failed"


@dataclass(frozen=True)
class CartesianTrajectoryPolicy:
    """Numerical, geometric and timing policy for one Cartesian plan.

    ``time_scaling`` controls the final ``JointTrajectory`` time line.  The
    geometry samples use the minimum-jerk progress curve, while execution
    derivatives are then validated against the exact quintic interpolation
    used by the motion executor.
    """

    path_mode: CartesianPathMode = CartesianPathMode.LINEAR_TRANSLATION_SLERP
    ik_policy: IKPolicy = field(default_factory=IKPolicy)
    time_scaling: TimeScalingPolicy = field(default_factory=TimeScalingPolicy)
    max_translation_step_m: float = 0.01
    max_rotation_step_rad: float = 0.08
    max_waypoints: int = 200
    maximum_joint_step_rad: float = 0.35
    dense_validation_sample_period_s: float = 0.02
    position_validation_tolerance_m: float = 0.003
    orientation_validation_tolerance_rad: float = 0.05
    max_duration_stretch_iterations: int = 8

    def __post_init__(self) -> None:
        path_mode = CartesianPathMode(self.path_mode)
        if not isinstance(self.ik_policy, IKPolicy):
            raise TypeError("ik_policy must be an IKPolicy")
        if not isinstance(self.time_scaling, TimeScalingPolicy):
            raise TypeError("time_scaling must be a TimeScalingPolicy")
        positive_values = {
            "max_translation_step_m": self.max_translation_step_m,
            "max_rotation_step_rad": self.max_rotation_step_rad,
            "maximum_joint_step_rad": self.maximum_joint_step_rad,
            "dense_validation_sample_period_s": self.dense_validation_sample_period_s,
            "position_validation_tolerance_m": self.position_validation_tolerance_m,
            "orientation_validation_tolerance_rad": self.orientation_validation_tolerance_rad,
        }
        for name, value in positive_values.items():
            normalized = float(value)
            if not math.isfinite(normalized) or normalized <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
            object.__setattr__(self, name, normalized)
        max_waypoints = int(self.max_waypoints)
        if max_waypoints < 2:
            raise ValueError("max_waypoints must be at least two")
        attempts = int(self.max_duration_stretch_iterations)
        if attempts < 0:
            raise ValueError("max_duration_stretch_iterations must not be negative")
        object.__setattr__(self, "path_mode", path_mode)
        object.__setattr__(self, "max_waypoints", max_waypoints)
        object.__setattr__(self, "max_duration_stretch_iterations", attempts)


@dataclass(frozen=True)
class CartesianTrajectoryPlanningRequest:
    """Pure planner input from an explicit fresh measured robot state."""

    q_start: JointPositions
    target_pose: Pose
    motion_limits: JointMotionLimits
    policy: CartesianTrajectoryPolicy = field(default_factory=CartesianTrajectoryPolicy)

    def __post_init__(self) -> None:
        if not isinstance(self.q_start, JointPositions):
            raise TypeError("q_start must be JointPositions")
        if not isinstance(self.target_pose, Pose):
            raise TypeError("target_pose must be Pose")
        if not isinstance(self.motion_limits, JointMotionLimits):
            raise TypeError("motion_limits must be JointMotionLimits")
        if not isinstance(self.policy, CartesianTrajectoryPolicy):
            raise TypeError("policy must be CartesianTrajectoryPolicy")


@dataclass(frozen=True)
class CartesianTrajectoryPlanningResult:
    """A fully validated Cartesian plan or an explicit safe rejection."""

    trajectory: Optional[JointTrajectory]
    succeeded: bool
    failure_reason: Optional[CartesianTrajectoryPlanningFailureReason]
    detail: str
    reference_path: Tuple[Pose, ...]
    requested_duration_s: Optional[float]
    minimum_duration_s: float
    resolved_duration_s: Optional[float]
    duration_adjusted: bool
    failed_waypoint_index: Optional[int]
    maximum_position_residual_m: float
    maximum_orientation_residual_rad: float
    minimum_singular_value: float
    minimum_joint_limit_margin_rad: float

    def __post_init__(self) -> None:
        reference_path = tuple(self.reference_path)
        if len(reference_path) < 2 or not all(
            isinstance(pose, Pose) for pose in reference_path
        ):
            raise ValueError("reference_path must contain at least two Pose values")
        minimum_duration_s = float(self.minimum_duration_s)
        if not math.isfinite(minimum_duration_s) or minimum_duration_s < 0.0:
            raise ValueError("minimum_duration_s must be finite and non-negative")
        object.__setattr__(self, "reference_path", reference_path)
        object.__setattr__(self, "minimum_duration_s", minimum_duration_s)
        if self.succeeded:
            if self.trajectory is None or self.failure_reason is not None:
                raise ValueError("successful result must contain only a trajectory")
            if self.resolved_duration_s is None:
                raise ValueError("successful result requires resolved_duration_s")
            resolved_duration_s = float(self.resolved_duration_s)
            if not math.isfinite(resolved_duration_s) or resolved_duration_s <= 0.0:
                raise ValueError("resolved_duration_s must be finite and positive")
            if not math.isclose(
                self.trajectory.duration_s,
                resolved_duration_s,
                rel_tol=0.0,
                abs_tol=1e-12,
            ):
                raise ValueError(
                    "trajectory final timestamp must equal resolved_duration_s"
                )
            object.__setattr__(self, "resolved_duration_s", resolved_duration_s)
        elif self.trajectory is not None or self.failure_reason is None:
            raise ValueError("failed result must contain a failure reason only")
        if self.failed_waypoint_index is not None:
            index = int(self.failed_waypoint_index)
            if index < 0 or index >= len(reference_path):
                raise ValueError("failed_waypoint_index must reference reference_path")
            object.__setattr__(self, "failed_waypoint_index", index)

    @property
    def success(self) -> bool:
        """Compatibility-friendly spelling for action and UI consumers."""
        return self.succeeded

    @property
    def waypoint_count(self) -> int:
        """Return the complete desired Cartesian reference sample count."""
        return len(self.reference_path)


def make_cartesian_reference_path(
    start_pose: Pose,
    target_pose: Pose,
    policy: CartesianTrajectoryPolicy,
) -> Tuple[Pose, ...]:
    """Create endpoint-inclusive minimum-jerk Cartesian references.

    The number of segments is chosen from the larger translation/rotation
    demand.  The geometry itself is sampled at minimum-jerk progress; it is
    later mapped to time by the Cartesian planner's validated joint profile.
    """
    translation_distance_m = pose_translation_distance_m(start_pose, target_pose)
    rotation_distance_rad = pose_orientation_distance_rad(start_pose, target_pose)
    translation_segments = math.ceil(
        translation_distance_m / policy.max_translation_step_m
    )
    rotation_segments = math.ceil(
        rotation_distance_rad / policy.max_rotation_step_rad
    )
    segments = max(1, translation_segments, rotation_segments)
    if segments + 1 > policy.max_waypoints:
        raise ValueError(
            f"Cartesian path requires {segments + 1} waypoints, exceeding max_waypoints={policy.max_waypoints}"
        )
    return tuple(
        interpolate_cartesian_pose(
            start_pose,
            target_pose,
            minimum_jerk_position_scale(index / segments),
            policy.path_mode,
        )
        for index in range(segments + 1)
    )


def interpolate_cartesian_pose(
    start_pose: Pose,
    target_pose: Pose,
    progress: float,
    path_mode: CartesianPathMode = CartesianPathMode.LINEAR_TRANSLATION_SLERP,
) -> Pose:
    """Interpolate two poses without ever converting orientation to Euler.

    ``linear_translation_slerp`` preserves a straight TCP translation.
    ``se3_geodesic`` follows the screw/geodesic interpolation used by the
    reBot reference implementation; it deliberately does *not* promise a
    straight XYZ path.
    """
    path_mode = CartesianPathMode(path_mode)
    scalar = _unit_interval(progress)
    if path_mode is CartesianPathMode.LINEAR_TRANSLATION_SLERP:
        return Pose(
            position=tuple(
                (1.0 - scalar) * start + scalar * target
                for start, target in zip(start_pose.position, target_pose.position)
            ),
            orientation=_slerp_xyzw(
                start_pose.orientation, target_pose.orientation, scalar
            ),
        )
    if path_mode is CartesianPathMode.SE3_GEODESIC:
        return _se3_geodesic(start_pose, target_pose, scalar)
    raise ValueError(f"unsupported Cartesian path mode: {path_mode}")


def pose_translation_distance_m(first: Pose, second: Pose) -> float:
    """Return Euclidean translation distance in metres."""
    return math.sqrt(
        sum((right - left) * (right - left) for left, right in zip(first.position, second.position))
    )


def pose_orientation_distance_rad(first: Pose, second: Pose) -> float:
    """Return shortest quaternion angular distance in radians."""
    dot = abs(
        sum(
            left * right
            for left, right in zip(first.orientation, second.orientation)
        )
    )
    return 2.0 * math.acos(max(-1.0, min(1.0, dot)))


def pose_se3_error_vector(
    actual_pose: Pose,
    desired_pose: Pose,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """Return local-frame translational and rotational SE(3) error vectors.

    The convention matches the Pinocchio IK error used by this SDK:
    ``actual.actInv(desired)`` followed by an SE(3) logarithm.  It gives dense
    Cartesian validation a component-wise, mask-aware error without Euler
    angles or a Pinocchio dependency in ``core``.
    """
    actual_rotation = _rotation_matrix_from_quaternion_xyzw(
        actual_pose.orientation
    )
    desired_rotation = _rotation_matrix_from_quaternion_xyzw(
        desired_pose.orientation
    )
    rotation_vector = _so3_log(actual_rotation.T.dot(desired_rotation))
    translation_local = actual_rotation.T.dot(
        np.asarray(desired_pose.position, dtype=float)
        - np.asarray(actual_pose.position, dtype=float)
    )
    try:
        translation_vector = np.linalg.solve(
            _so3_left_jacobian(rotation_vector), translation_local
        )
    except np.linalg.LinAlgError as error:
        raise ValueError("SE(3) error logarithm is numerically singular") from error
    return (
        tuple(float(value) for value in translation_vector),
        tuple(float(value) for value in rotation_vector),
    )


def _slerp_xyzw(
    first: Sequence[float], second: Sequence[float], progress: float
) -> Tuple[float, float, float, float]:
    left = np.asarray(first, dtype=float)
    right = np.asarray(second, dtype=float)
    dot = float(np.dot(left, right))
    if dot < 0.0:
        right = -right
        dot = -dot
    dot = max(-1.0, min(1.0, dot))
    if dot > 0.9995:
        result = left + progress * (right - left)
    else:
        theta = math.acos(dot)
        sin_theta = math.sin(theta)
        result = (
            math.sin((1.0 - progress) * theta) / sin_theta * left
            + math.sin(progress * theta) / sin_theta * right
        )
    result /= np.linalg.norm(result)
    return tuple(float(value) for value in result)


def _se3_geodesic(start_pose: Pose, target_pose: Pose, progress: float) -> Pose:
    start_rotation = _rotation_matrix_from_quaternion_xyzw(start_pose.orientation)
    target_rotation = _rotation_matrix_from_quaternion_xyzw(target_pose.orientation)
    relative_rotation = start_rotation.T.dot(target_rotation)
    rotation_vector = _so3_log(relative_rotation)
    translation_relative = start_rotation.T.dot(
        np.asarray(target_pose.position, dtype=float)
        - np.asarray(start_pose.position, dtype=float)
    )
    try:
        translational_twist = np.linalg.solve(
            _so3_left_jacobian(rotation_vector), translation_relative
        )
    except np.linalg.LinAlgError as error:
        raise ValueError("SE(3) path logarithm is numerically singular") from error
    scaled_rotation_vector = progress * rotation_vector
    scaled_translation = _so3_left_jacobian(scaled_rotation_vector).dot(
        progress * translational_twist
    )
    rotation = start_rotation.dot(_so3_exp(scaled_rotation_vector))
    position = np.asarray(start_pose.position, dtype=float) + start_rotation.dot(
        scaled_translation
    )
    return Pose(
        position=tuple(float(value) for value in position),
        orientation=_quaternion_xyzw_from_rotation_matrix(rotation),
    )


def _rotation_matrix_from_quaternion_xyzw(quaternion: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(value) for value in quaternion)
    return np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _quaternion_xyzw_from_rotation_matrix(rotation: np.ndarray) -> Tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(max(0.0, 1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2.0
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
        w = (rotation[2, 1] - rotation[1, 2]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(max(0.0, 1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2.0
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
        w = (rotation[0, 2] - rotation[2, 0]) / scale
    else:
        scale = math.sqrt(max(0.0, 1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2.0
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
        w = (rotation[1, 0] - rotation[0, 1]) / scale
    quaternion = np.array((x, y, z, w), dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return tuple(float(value) for value in quaternion)


def _so3_exp(rotation_vector: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotation_vector))
    matrix = _hat(rotation_vector)
    if theta < 1e-9:
        return np.eye(3) + matrix + 0.5 * matrix.dot(matrix)
    theta2 = theta * theta
    return (
        np.eye(3)
        + math.sin(theta) / theta * matrix
        + (1.0 - math.cos(theta)) / theta2 * matrix.dot(matrix)
    )


def _so3_log(rotation: np.ndarray) -> np.ndarray:
    cosine = max(-1.0, min(1.0, (float(np.trace(rotation)) - 1.0) * 0.5))
    theta = math.acos(cosine)
    if theta < 1e-8:
        return _vee(rotation - rotation.T) * 0.5
    if math.pi - theta < 1e-5:
        diagonal = np.maximum(np.diag(rotation) + 1.0, 0.0)
        axis = np.sqrt(diagonal * 0.5)
        if axis[0] > 1e-7:
            axis[1] = math.copysign(axis[1], rotation[0, 1] + rotation[1, 0])
            axis[2] = math.copysign(axis[2], rotation[0, 2] + rotation[2, 0])
        elif axis[1] > 1e-7:
            axis[2] = math.copysign(axis[2], rotation[1, 2] + rotation[2, 1])
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            raise ValueError("rotation logarithm is undefined for this matrix")
        return theta * axis / norm
    return theta / (2.0 * math.sin(theta)) * _vee(rotation - rotation.T)


def _so3_left_jacobian(rotation_vector: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotation_vector))
    matrix = _hat(rotation_vector)
    if theta < 1e-8:
        return np.eye(3) + 0.5 * matrix + matrix.dot(matrix) / 6.0
    theta2 = theta * theta
    return (
        np.eye(3)
        + (1.0 - math.cos(theta)) / theta2 * matrix
        + (theta - math.sin(theta)) / (theta2 * theta) * matrix.dot(matrix)
    )


def _hat(vector: np.ndarray) -> np.ndarray:
    x, y, z = (float(value) for value in vector)
    return np.array(((0.0, -z, y), (z, 0.0, -x), (-y, x, 0.0)), dtype=float)


def _vee(matrix: np.ndarray) -> np.ndarray:
    return np.array((matrix[2, 1], matrix[0, 2], matrix[1, 0]), dtype=float)


def _unit_interval(value: float) -> float:
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError("progress must be finite")
    return max(0.0, min(1.0, normalized))
