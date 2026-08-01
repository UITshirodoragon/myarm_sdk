"""Framework-free loading of canonical actuated-joint metadata from URDF."""

from __future__ import annotations

import math
import xml.etree.ElementTree as element_tree
from pathlib import Path
from typing import Sequence, Tuple, Union

from .joint_metadata import JointMetadata


def load_urdf_joint_metadata(
    urdf_path: Union[str, Path], joint_names: Sequence[str]
) -> Tuple[JointMetadata, ...]:
    """Load ordered revolute-joint metadata from one authoritative URDF.

    This deliberately depends only on the Python standard library.  Hardware,
    kinematics and planning code can therefore share its canonical joint order,
    right-hand-rule axes and hard limits without importing ROS or Pinocchio.
    ``joint_names`` supplies the required model order; it is not inferred from
    XML declaration order.
    """
    path = Path(urdf_path)
    if not path.is_file():
        raise ValueError(f"URDF file does not exist: {path}")
    names = _validate_joint_names(joint_names)
    try:
        root = element_tree.parse(str(path)).getroot()
    except element_tree.ParseError as error:
        raise ValueError(f"URDF is not valid XML: {path}") from error

    joint_elements = {}
    for element in root.findall("joint"):
        name = element.attrib.get("name")
        if not name:
            raise ValueError("URDF joint is missing a name")
        if name in joint_elements:
            raise ValueError(f"URDF contains duplicate joint name: {name}")
        joint_elements[name] = element

    metadata = []
    for name in names:
        element = joint_elements.get(name)
        if element is None:
            raise ValueError(f"URDF is missing joint metadata for {name}")
        if element.attrib.get("type") != "revolute":
            raise ValueError(f"URDF arm joint {name} must be revolute")
        axis_element = element.find("axis")
        limit_element = element.find("limit")
        if axis_element is None or limit_element is None:
            raise ValueError(f"URDF arm joint {name} requires axis and limit")
        lower_limit_rad = _finite_attribute(limit_element, "lower", name)
        upper_limit_rad = _finite_attribute(limit_element, "upper", name)
        velocity_limit_rad_s = _finite_attribute(limit_element, "velocity", name)
        if lower_limit_rad >= upper_limit_rad:
            raise ValueError(f"URDF joint {name} lower limit must be below upper limit")
        if velocity_limit_rad_s <= 0.0:
            raise ValueError(f"URDF joint {name} velocity limit must be positive")
        metadata.append(
            JointMetadata(
                name=name,
                axis_xyz=_normalized_axis(
                    str(axis_element.attrib.get("xyz", "1 0 0")), name
                ),
                lower_limit_rad=lower_limit_rad,
                upper_limit_rad=upper_limit_rad,
                velocity_limit_rad_s=velocity_limit_rad_s,
            )
        )
    return tuple(metadata)


def _validate_joint_names(joint_names: Sequence[str]) -> Tuple[str, ...]:
    if isinstance(joint_names, str):
        raise TypeError("joint_names must be a sequence of joint names")
    names = tuple(joint_names)
    if not names:
        raise ValueError("joint_names must not be empty")
    if not all(isinstance(name, str) and name.strip() for name in names):
        raise ValueError("joint_names must contain non-empty strings")
    if len(set(names)) != len(names):
        raise ValueError("joint_names must be unique")
    return names


def _finite_attribute(element, attribute_name: str, joint_name: str) -> float:
    value = element.attrib.get(attribute_name)
    if value is None:
        raise ValueError(
            f"URDF joint {joint_name} limit is missing {attribute_name}"
        )
    try:
        normalized = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"URDF joint {joint_name} limit {attribute_name} must be numeric"
        ) from error
    if not math.isfinite(normalized):
        raise ValueError(
            f"URDF joint {joint_name} limit {attribute_name} must be finite"
        )
    return normalized


def _normalized_axis(axis_text: str, joint_name: str) -> Tuple[float, float, float]:
    try:
        axis = tuple(float(value) for value in axis_text.split())
    except ValueError as error:
        raise ValueError(f"URDF joint {joint_name} axis must be numeric") from error
    if len(axis) != 3:
        raise ValueError(f"URDF joint {joint_name} axis must have three values")
    if not all(math.isfinite(value) for value in axis):
        raise ValueError(f"URDF joint {joint_name} axis must be finite")
    norm = math.sqrt(sum(value * value for value in axis))
    if norm < 1e-12:
        raise ValueError(f"URDF joint {joint_name} axis must not be zero")
    return tuple(value / norm for value in axis)  # type: ignore[return-value]
