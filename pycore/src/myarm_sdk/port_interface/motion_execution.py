"""Timed joint-trajectory execution contract.

Implementations are deliberately transport-free: they sample a trajectory
against a monotonic clock and never open a serial port or publish ROS data.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Optional, Protocol

from myarm_sdk.core.joint_positions import JointPositions
from myarm_sdk.core.motion_execution import (
    MotionExecutionEvent,
    MotionExecutionFailureReason,
    MotionExecutionPolicy,
    MotionExecutionResult,
    MotionExecutionState,
)

if TYPE_CHECKING:
    from myarm_sdk.core.joint_trajectory import JointTrajectory


class MotionExecutionInterface(Protocol):
    """Advance one validated trajectory using a monotonic clock."""

    @property
    def state(self) -> MotionExecutionState:
        """Return the current execution lifecycle state."""
        ...

    def start(
        self,
        trajectory: JointTrajectory,
        policy: Optional[MotionExecutionPolicy] = None,
        now_monotonic_s: Optional[float] = None,
    ) -> MotionExecutionResult:
        """Start a new trajectory only when the executor is not active."""
        ...

    def tick(
        self,
        now_monotonic_s: Optional[float] = None,
        actual_positions: Optional[JointPositions] = None,
    ) -> MotionExecutionEvent:
        """Sample the active trajectory without blocking or performing I/O."""
        ...

    def cancel(
        self, hold_position: Optional[JointPositions] = None
    ) -> MotionExecutionResult:
        """Cancel an active motion and retain a non-I/O hold setpoint."""
        ...

    def hold(
        self, hold_position: Optional[JointPositions] = None
    ) -> MotionExecutionResult:
        """Stop trajectory progression and retain a non-I/O hold setpoint."""
        ...

    def fault(
        self,
        reason: MotionExecutionFailureReason,
        detail: str = "",
        hold_position: Optional[JointPositions] = None,
    ) -> MotionExecutionResult:
        """Put the executor into a fault state without commanding hardware."""
        ...

    def reset(self) -> MotionExecutionResult:
        """Acknowledge a terminal state and return to ``IDLE``."""
        ...
