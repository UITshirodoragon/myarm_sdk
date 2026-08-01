"""ROS-independent state and result types for timed motion execution.

The motion executor intentionally knows nothing about a robot transport.  It
only turns a validated joint trajectory and a monotonic clock into desired
joint setpoints.  A ROS boundary can then forward those setpoints to the one
robot driver that owns the physical connection.
"""

from __future__ import annotations

import enum
import math
from dataclasses import dataclass
from typing import Optional

from .joint_positions import JointPositions


class MotionExecutionState(str, enum.Enum):
    """Lifecycle of one motion execution request."""

    IDLE = "idle"
    EXECUTING = "executing"
    HOLDING = "holding"
    CANCELED = "canceled"
    SUCCEEDED = "succeeded"
    FAULT = "fault"


class MotionExecutionFailureReason(str, enum.Enum):
    """Why an execution was rejected, stopped, or faulted."""

    INVALID_TRAJECTORY = "invalid_trajectory"
    EXECUTION_ACTIVE = "execution_active"
    INVALID_STATE = "invalid_state"
    CLOCK_REGRESSION = "clock_regression"
    TICK_LAG = "tick_lag"
    TRACKING_ERROR = "tracking_error"
    TIMEOUT = "timeout"
    CANCELED = "canceled"
    HELD = "held"
    EXTERNAL_FAULT = "external_fault"


class MotionExecutionViolationAction(str, enum.Enum):
    """State transition selected when a timing or tracking policy is violated."""

    REPORT = "report"
    HOLD = "hold"
    FAULT = "fault"


@dataclass(frozen=True)
class MotionExecutionPolicy:
    """Policy for observing timing and feedback without controlling hardware.

    ``max_tick_lag_s`` measures the interval between two executor ticks.  It
    helps the ROS integration detect that its timer stopped making progress;
    it is unrelated to robot feedback.  ``max_tracking_error_rad`` is checked
    only when the caller supplies measured joints to :meth:`tick`.
    """

    completion_tolerance_rad: float = 0.05
    require_feedback_for_completion: bool = False
    timeout_margin_s: float = 1.0
    max_tick_lag_s: Optional[float] = None
    tick_lag_action: MotionExecutionViolationAction = (
        MotionExecutionViolationAction.REPORT
    )
    max_tracking_error_rad: Optional[float] = None
    tracking_error_action: MotionExecutionViolationAction = (
        MotionExecutionViolationAction.REPORT
    )
    timeout_action: MotionExecutionViolationAction = MotionExecutionViolationAction.HOLD

    def __post_init__(self) -> None:
        self._positive_finite(
            self.completion_tolerance_rad, "completion_tolerance_rad", allow_zero=True
        )
        if not isinstance(self.require_feedback_for_completion, bool):
            raise TypeError("require_feedback_for_completion must be boolean")
        self._positive_finite(
            self.timeout_margin_s, "timeout_margin_s", allow_zero=True
        )
        self._optional_positive_finite(self.max_tick_lag_s, "max_tick_lag_s")
        self._optional_positive_finite(
            self.max_tracking_error_rad, "max_tracking_error_rad"
        )
        self._validate_action(self.tick_lag_action, "tick_lag_action")
        self._validate_action(self.tracking_error_action, "tracking_error_action")
        self._validate_action(self.timeout_action, "timeout_action")

    @staticmethod
    def _positive_finite(value: float, name: str, allow_zero: bool = False) -> None:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric, not boolean")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(number) or (number < 0.0 if allow_zero else number <= 0.0):
            comparator = "non-negative" if allow_zero else "positive"
            raise ValueError(f"{name} must be finite and {comparator}")

    @classmethod
    def _optional_positive_finite(cls, value: Optional[float], name: str) -> None:
        if value is not None:
            cls._positive_finite(value, name)

    @staticmethod
    def _validate_action(value: MotionExecutionViolationAction, name: str) -> None:
        if not isinstance(value, MotionExecutionViolationAction):
            raise TypeError(f"{name} must be MotionExecutionViolationAction")


@dataclass(frozen=True)
class MotionExecutionSetpoint:
    """Desired joint state sampled at an exact trajectory time."""

    positions: JointPositions
    velocities: JointPositions
    accelerations: JointPositions
    time_from_start_s: float

    def __post_init__(self) -> None:
        if isinstance(self.time_from_start_s, bool):
            raise TypeError("time_from_start_s must be numeric, not boolean")
        try:
            time_s = float(self.time_from_start_s)
        except (TypeError, ValueError) as error:
            raise TypeError("time_from_start_s must be numeric") from error
        if not math.isfinite(time_s) or time_s < 0.0:
            raise ValueError("time_from_start_s must be finite and non-negative")
        object.__setattr__(self, "time_from_start_s", time_s)


@dataclass(frozen=True)
class MotionExecutionResult:
    """Outcome of a state-changing executor operation."""

    accepted: bool
    state: MotionExecutionState
    reason: Optional[MotionExecutionFailureReason] = None
    detail: str = ""


@dataclass(frozen=True)
class MotionExecutionEvent:
    """One non-blocking execution update returned by :meth:`tick`."""

    state: MotionExecutionState
    desired_setpoint: Optional[MotionExecutionSetpoint]
    elapsed_s: Optional[float]
    duration_s: Optional[float]
    progress: float
    tracking_error: Optional[JointPositions]
    max_tracking_error_rad: Optional[float]
    tick_lag_s: Optional[float]
    lagged: bool
    timed_out: bool
    reason: Optional[MotionExecutionFailureReason] = None
    detail: str = ""

    def __post_init__(self) -> None:
        if not 0.0 <= self.progress <= 1.0:
            raise ValueError("progress must be in the range [0, 1]")
