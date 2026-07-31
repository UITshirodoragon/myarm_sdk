"""Spatial conversions at the pytransform3d boundary."""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np


def _rotations_module():
    """Load the optional conversion dependency only when it is needed."""
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
    """Convert a ROS-style ``(x, y, z, w)`` quaternion into a 3x3 matrix."""
    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if quaternion.shape != (4,) or not np.isfinite(quaternion).all():
        raise ValueError("orientation must contain four finite quaternion values")

    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("orientation quaternion must not have zero length")

    quaternion /= norm
    return _rotations_module().matrix_from_quaternion(
        np.array(
            [quaternion[3], quaternion[0], quaternion[1], quaternion[2]],
            dtype=float,
        )
    )


def quaternion_xyzw_from_rotation_matrix(
    rotation_matrix: Sequence[Sequence[float]],
) -> Tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix into a ROS-style ``(x, y, z, w)`` quaternion."""
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
