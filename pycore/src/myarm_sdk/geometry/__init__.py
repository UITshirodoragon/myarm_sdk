"""Geometry conversions shared by kinematics adapters.

This package is the only pycore boundary allowed to depend on
``pytransform3d``.  Public SDK poses remain metre and ROS-style ``xyzw``
quaternions.
"""

from .conversions import (
    quaternion_xyzw_from_rotation_matrix,
    rotation_matrix_from_quaternion_xyzw,
)

__all__ = [
    "quaternion_xyzw_from_rotation_matrix",
    "rotation_matrix_from_quaternion_xyzw",
]
