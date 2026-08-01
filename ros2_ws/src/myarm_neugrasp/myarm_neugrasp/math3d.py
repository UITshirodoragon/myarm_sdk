"""Small dependency-free SE(3) helpers used by NeuGrasp ROS boundaries."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


Vector3 = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


@dataclass(frozen=True)
class RigidTransform:
    """Rigid transform represented as target_T_source."""

    translation: Vector3
    rotation: Quaternion


def finite_vector(values: Sequence[float], name: str, size: int) -> Tuple[float, ...]:
    if len(values) != size:
        raise ValueError(f"{name} must contain exactly {size} values")
    try:
        normalized = tuple(float(value) for value in values)
    except (TypeError, ValueError) as error:
        raise TypeError(f"{name} must be numeric") from error
    if not all(math.isfinite(value) for value in normalized):
        raise ValueError(f"{name} must be finite")
    return normalized


def normalize_quaternion(values: Sequence[float]) -> Quaternion:
    x, y, z, w = finite_vector(values, "quaternion", 4)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError("quaternion must not be zero")
    return x / norm, y / norm, z / norm, w / norm


def quaternion_conjugate(quaternion: Quaternion) -> Quaternion:
    x, y, z, w = quaternion
    return -x, -y, -z, w


def quaternion_multiply(left: Quaternion, right: Quaternion) -> Quaternion:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    x, y, z, w = normalize_quaternion(quaternion)
    vx, vy, vz = finite_vector(vector, "vector", 3)
    # Equivalent to q * [v, 0] * conjugate(q), expanded to avoid a zero-norm
    # pseudo-quaternion through quaternion_multiply.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compose(left: RigidTransform, right: RigidTransform) -> RigidTransform:
    """Return target_T_intermediate × intermediate_T_source."""
    right_translation_in_target = rotate_vector(left.rotation, right.translation)
    return RigidTransform(
        translation=(
            left.translation[0] + right_translation_in_target[0],
            left.translation[1] + right_translation_in_target[1],
            left.translation[2] + right_translation_in_target[2],
        ),
        rotation=quaternion_multiply(left.rotation, right.rotation),
    )


def inverse(transform: RigidTransform) -> RigidTransform:
    inverse_rotation = quaternion_conjugate(normalize_quaternion(transform.rotation))
    negated = (-transform.translation[0], -transform.translation[1], -transform.translation[2])
    return RigidTransform(
        translation=rotate_vector(inverse_rotation, negated),
        rotation=inverse_rotation,
    )


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return left[0] - right[0], left[1] - right[1], left[2] - right[2]


def _normalize(vector: Vector3, name: str) -> Vector3:
    x, y, z = finite_vector(vector, name, 3)
    norm = math.sqrt(x * x + y * y + z * z)
    if norm < 1e-12:
        raise ValueError(f"{name} must not be zero")
    return x / norm, y / norm, z / norm


def quaternion_from_matrix(columns: Tuple[Vector3, Vector3, Vector3]) -> Quaternion:
    """Return the quaternion for a rotation matrix specified by columns."""
    x_axis, y_axis, z_axis = columns
    m00, m10, m20 = x_axis
    m01, m11, m21 = y_axis
    m02, m12, m22 = z_axis
    trace = m00 + m11 + m22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            (m21 - m12) / scale,
            (m02 - m20) / scale,
            (m10 - m01) / scale,
            0.25 * scale,
        )
    elif m00 > m11 and m00 > m22:
        scale = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        quaternion = (
            0.25 * scale,
            (m01 + m10) / scale,
            (m02 + m20) / scale,
            (m21 - m12) / scale,
        )
    elif m11 > m22:
        scale = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        quaternion = (
            (m01 + m10) / scale,
            0.25 * scale,
            (m12 + m21) / scale,
            (m02 - m20) / scale,
        )
    else:
        scale = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        quaternion = (
            (m02 + m20) / scale,
            (m12 + m21) / scale,
            0.25 * scale,
            (m10 - m01) / scale,
        )
    return normalize_quaternion(quaternion)


def optical_look_at(position: Vector3, target: Vector3) -> Quaternion:
    """Orient a REP-103 optical frame so +Z looks at ``target``.

    The camera's +Y axis points down as far as possible relative to world +Z.
    """
    forward = _normalize(_subtract(target, position), "look-at direction")
    world_up = (0.0, 0.0, 1.0)
    right = _cross(forward, world_up)
    try:
        right = _normalize(right, "camera right axis")
    except ValueError:
        right = _normalize(_cross(forward, (0.0, 1.0, 0.0)), "camera right axis")
    down = _normalize(_cross(forward, right), "camera down axis")
    return quaternion_from_matrix((right, down, forward))
