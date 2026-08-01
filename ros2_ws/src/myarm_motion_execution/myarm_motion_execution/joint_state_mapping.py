"""Strict ROS-boundary mapping for canonical MyArm joint positions."""

from __future__ import annotations

from typing import Sequence, Tuple

from myarm_sdk.core import JointPositions


def canonical_joint_positions_from_names(
    names: Sequence[str],
    positions: Sequence[float],
    canonical_joint_names: Sequence[str],
) -> JointPositions:
    """Map a ``JointState`` into the six canonical arm joints.

    Named messages may carry additional joints such as the gripper; all six
    arm joints must still occur exactly once.  Unnamed messages are accepted
    only if their positions are already in canonical order.  Keeping this
    conversion at the ROS boundary prevents a malformed message from being
    silently interpreted as a different arm pose by pycore.
    """
    expected_names = _canonical_names(canonical_joint_names)
    received_names = tuple(names)
    received_positions = tuple(positions)
    if received_names:
        if len(received_names) != len(received_positions):
            raise ValueError("JointState name and position lengths differ")
        if not all(isinstance(name, str) and name for name in received_names):
            raise ValueError("JointState names must be non-empty strings")
        if len(set(received_names)) != len(received_names):
            raise ValueError("JointState contains duplicate joint names")
        positions_by_name = dict(zip(received_names, received_positions))
        missing = [name for name in expected_names if name not in positions_by_name]
        if missing:
            raise ValueError(
                "JointState is missing canonical arm joints: {}".format(
                    ", ".join(missing)
                )
            )
        return JointPositions(
            tuple(positions_by_name[name] for name in expected_names)
        )

    if len(received_positions) != len(expected_names):
        raise ValueError(
            f"unnamed JointState must contain exactly {len(expected_names)} canonical arm positions"
        )
    return JointPositions(received_positions)


def _canonical_names(joint_names: Sequence[str]) -> Tuple[str, ...]:
    names = tuple(joint_names)
    if len(names) != 6:
        raise ValueError("MyArm M750 canonical joint order must contain six names")
    if not all(isinstance(name, str) and name for name in names):
        raise ValueError("canonical joint names must be non-empty strings")
    if len(set(names)) != len(names):
        raise ValueError("canonical joint names must be unique")
    return names
