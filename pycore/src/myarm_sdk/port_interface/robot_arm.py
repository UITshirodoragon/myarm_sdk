"""Stateful, hardware-neutral robot-arm contract."""

from __future__ import annotations

from typing import Protocol, Tuple

from myarm_sdk.core import (
    GripperCommand,
    GripperState,
    JointPositions,
    RobotArmCommand,
    RobotArmState,
)


class RobotArmInterface(Protocol):
    """Read and command one six-axis robot in canonical model coordinates.

    The ``state`` property is a cached immutable snapshot and never performs
    I/O.  ``read_state`` is the explicit feedback transaction.  A concrete
    implementation must not replace measured state with a target merely
    because a physical command was acknowledged.

    ``JointPositions`` is intentionally an ordered value-only type.  A ROS
    transport boundary must map incoming joint names into ``joint_names``
    before calling this interface.
    """

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """Return the canonical URDF joint order."""

    @property
    def state(self) -> RobotArmState:
        """Return the latest cached state without device I/O."""

    @property
    def is_connected(self) -> bool:
        """Return whether this backend currently has an active connection."""

    def connect(self) -> RobotArmState:
        """Connect the backend without implicitly commanding motion."""

    def disconnect(self) -> RobotArmState:
        """Release backend resources without implicitly powering off hardware."""

    def read_state(self) -> RobotArmState:
        """Read measured canonical joint positions and update the cache."""

    def write_joint_positions(
        self, target: JointPositions, speed_scale: float = 0.5
    ) -> RobotArmCommand:
        """Send one validated canonical joint-position target."""

    def stop(self) -> RobotArmState:
        """Request a software motion stop from the backend."""

    def power_on(self) -> RobotArmState:
        """Request power/servo enable and verify the resulting state."""

    def power_off(self) -> RobotArmState:
        """Request power/servo disable and verify the resulting state."""

    def read_power_state(self) -> RobotArmState:
        """Read and cache the backend power state."""

    def read_motion_state(self) -> RobotArmState:
        """Read and cache whether the backend reports active motion."""

    def read_gripper_state(self) -> GripperState:
        """Read cached physical gripper opening in total fingertip metres."""

    def enable_gripper(self) -> RobotArmState:
        """Enable gripper actuation without moving either jaw."""

    def write_gripper_opening(
        self, opening_width_m: float, speed_scale: float = 0.5
    ) -> GripperCommand:
        """Command total fingertip opening in the range [0, 0.08] metres."""

    def read_gripper_motion_state(self) -> GripperState:
        """Read and cache whether the gripper reports active motion."""
