"""Conversions between SDK/ROS and matrix rotation representations."""

from typing import Sequence, Tuple

import numpy as np


def _rotations_module():
    """Import pytransform3d lazily so the base SDK stays usable without it."""
    try:
        from pytransform3d import rotations
    except ImportError as error:
        raise RuntimeError(
            "Install kinematics support with `pip install myarm-sdk[kinematics]`."
        ) from error
    return rotations


def rotation_matrix_from_quaternion_xyzw(
    quaternion_xyzw: Sequence[float],
) -> np.ndarray:
    """Return a 3x3 rotation matrix from a normalized ROS ``(x, y, z, w)`` quaternion."""
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("orientation must contain four finite quaternion values")

    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("orientation quaternion must not have zero length")

    quaternion /= norm
    # pytransform3d uses scalar-first quaternions: (w, x, y, z).
    quaternion_wxyz = np.array(
        [quaternion[3], quaternion[0], quaternion[1], quaternion[2]], dtype=float
    )
    return _rotations_module().matrix_from_quaternion(quaternion_wxyz)


def quaternion_xyzw_from_rotation_matrix(
    rotation_matrix: Sequence[Sequence[float]],
) -> Tuple[float, float, float, float]:
    """Return a normalized ROS ``(x, y, z, w)`` quaternion from a 3x3 matrix."""
    matrix = np.asarray(rotation_matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError("rotation matrix must be a finite 3x3 matrix")

    quaternion_wxyz = _rotations_module().quaternion_from_matrix(matrix)
    return (
        float(quaternion_wxyz[1]),
        float(quaternion_wxyz[2]),
        float(quaternion_wxyz[3]),
        float(quaternion_wxyz[0]),
    )
