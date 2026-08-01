"""Pure monotonic-clock trajectory execution adapter.

This adapter owns no robot transport.  It is deliberately usable in unit
tests, fake-robot runs and ROS integrations alike: callers supply a monotonic
time and optionally measured joints, then forward the returned setpoint to a
separate driver boundary.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Optional, Tuple

from myarm_sdk.core.joint_trajectory_interpolation import (
    sample_joint_trajectory,
)
from myarm_sdk.core.joint_positions import JointPositions
from myarm_sdk.core.motion_execution import (
    MotionExecutionEvent,
    MotionExecutionFailureReason,
    MotionExecutionPolicy,
    MotionExecutionResult,
    MotionExecutionSetpoint,
    MotionExecutionState,
    MotionExecutionViolationAction,
)


class MonotonicTimeMotionExecutionAdapter:
    """Sample one prevalidated joint trajectory against :func:`time.monotonic`.

    Starting another trajectory while one is active is rejected.  This mirrors
    the useful state-gating idea from reBot's action server, while keeping the
    executor non-blocking and independent from ROS and hardware.
    """

    _JOINT_COUNT = 6
    _TIME_EPSILON_S = 1e-9

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._state = MotionExecutionState.IDLE
        self._trajectory: Optional[Any] = None
        self._policy = MotionExecutionPolicy()
        self._started_at_s: Optional[float] = None
        self._last_tick_at_s: Optional[float] = None
        self._last_elapsed_s: Optional[float] = None
        self._last_setpoint: Optional[MotionExecutionSetpoint] = None
        self._hold_setpoint: Optional[MotionExecutionSetpoint] = None
        self._last_tracking_error: Optional[JointPositions] = None
        self._last_max_tracking_error_rad: Optional[float] = None
        self._reason: Optional[MotionExecutionFailureReason] = None
        self._detail = ""

    @property
    def state(self) -> MotionExecutionState:
        """Return the current lifecycle state without advancing time."""
        with self._lock:
            return self._state

    def start(
        self,
        trajectory: Any,
        policy: Optional[MotionExecutionPolicy] = None,
        now_monotonic_s: Optional[float] = None,
    ) -> MotionExecutionResult:
        """Start a trajectory from time zero when the state permits it.

        The input is structurally validated even if it was created by a
        planner, because this adapter is the final pure-Python boundary before
        execution.  It does not perform I/O.
        """
        with self._lock:
            if self._state == MotionExecutionState.EXECUTING:
                return MotionExecutionResult(
                    accepted=False,
                    state=self._state,
                    reason=MotionExecutionFailureReason.EXECUTION_ACTIVE,
                    detail="an execution is already active; cancel or hold it first",
                )
            if self._state in (
                MotionExecutionState.HOLDING,
                MotionExecutionState.FAULT,
            ):
                return MotionExecutionResult(
                    accepted=False,
                    state=self._state,
                    reason=MotionExecutionFailureReason.INVALID_STATE,
                    detail=f"reset is required before starting from {self._state.value}",
                )

            selected_policy = policy if policy is not None else MotionExecutionPolicy()
            if not isinstance(selected_policy, MotionExecutionPolicy):
                raise TypeError("policy must be MotionExecutionPolicy or None")
            try:
                self._validate_trajectory(trajectory)
                start_time_s = self._normalize_monotonic_time(now_monotonic_s)
            except (TypeError, ValueError) as error:
                return MotionExecutionResult(
                    accepted=False,
                    state=self._state,
                    reason=MotionExecutionFailureReason.INVALID_TRAJECTORY,
                    detail=str(error),
                )

            self._trajectory = trajectory
            self._policy = selected_policy
            self._started_at_s = start_time_s
            self._last_tick_at_s = None
            self._last_elapsed_s = 0.0
            self._last_setpoint = self._sample(trajectory, 0.0)
            self._hold_setpoint = None
            self._last_tracking_error = None
            self._last_max_tracking_error_rad = None
            self._reason = None
            self._detail = ""
            self._state = MotionExecutionState.EXECUTING
            return MotionExecutionResult(
                accepted=True,
                state=self._state,
                detail="trajectory execution started",
            )

    def tick(
        self,
        now_monotonic_s: Optional[float] = None,
        actual_positions: Optional[JointPositions] = None,
    ) -> MotionExecutionEvent:
        """Return the current desired setpoint without sleeping or writing I/O."""
        with self._lock:
            now_s = self._normalize_monotonic_time(now_monotonic_s)
            if actual_positions is not None and not isinstance(
                actual_positions, JointPositions
            ):
                raise TypeError("actual_positions must be JointPositions or None")

            if self._state != MotionExecutionState.EXECUTING:
                return self._terminal_event()

            if self._last_tick_at_s is not None and now_s < self._last_tick_at_s:
                self._transition_to_fault(
                    MotionExecutionFailureReason.CLOCK_REGRESSION,
                    "monotonic time moved backwards",
                    self._last_setpoint,
                )
                return self._terminal_event()

            if self._started_at_s is None or self._trajectory is None:
                self._transition_to_fault(
                    MotionExecutionFailureReason.EXTERNAL_FAULT,
                    "active execution has no trajectory state",
                    self._last_setpoint,
                )
                return self._terminal_event()

            elapsed_s = max(0.0, now_s - self._started_at_s)
            self._last_elapsed_s = elapsed_s
            duration_s = self._trajectory_duration_s(self._trajectory)
            sampled_time_s = min(elapsed_s, duration_s)
            setpoint = self._sample(self._trajectory, sampled_time_s)
            self._last_setpoint = setpoint

            tick_lag_s = self._tick_lag_s(now_s)
            self._last_tick_at_s = now_s
            lagged = (
                tick_lag_s is not None
                and self._policy.max_tick_lag_s is not None
                and tick_lag_s > self._policy.max_tick_lag_s
            )
            event_reason: Optional[MotionExecutionFailureReason] = None
            event_detail = ""
            if lagged:
                detail = f"executor tick gap {tick_lag_s:.6f}s exceeds policy {self._policy.max_tick_lag_s:.6f}s"
                event_reason = MotionExecutionFailureReason.TICK_LAG
                event_detail = detail
                if self._apply_violation(
                    self._policy.tick_lag_action,
                    MotionExecutionFailureReason.TICK_LAG,
                    detail,
                    setpoint,
                ):
                    return self._terminal_event(
                        elapsed_s=elapsed_s,
                        duration_s=duration_s,
                        tick_lag_s=tick_lag_s,
                        lagged=True,
                    )

            tracking_error, max_tracking_error_rad = self._tracking_error(
                setpoint.positions, actual_positions
            )
            self._last_tracking_error = tracking_error
            self._last_max_tracking_error_rad = max_tracking_error_rad
            if (
                max_tracking_error_rad is not None
                and self._policy.max_tracking_error_rad is not None
                and max_tracking_error_rad > self._policy.max_tracking_error_rad
            ):
                detail = f"tracking error {max_tracking_error_rad:.6f} rad exceeds policy {self._policy.max_tracking_error_rad:.6f} rad"
                event_reason = MotionExecutionFailureReason.TRACKING_ERROR
                event_detail = detail
                if self._apply_violation(
                    self._policy.tracking_error_action,
                    MotionExecutionFailureReason.TRACKING_ERROR,
                    detail,
                    setpoint,
                ):
                    return self._terminal_event(
                        elapsed_s=elapsed_s,
                        duration_s=duration_s,
                        tick_lag_s=tick_lag_s,
                        lagged=lagged,
                    )

            timed_out = False
            if elapsed_s >= duration_s:
                if (
                    actual_positions is not None
                    and max_tracking_error_rad is not None
                    and max_tracking_error_rad <= self._policy.completion_tolerance_rad
                ) or (
                    actual_positions is None
                    and not self._policy.require_feedback_for_completion
                ):
                    self._state = MotionExecutionState.SUCCEEDED
                    self._reason = None
                    self._detail = "trajectory reached its terminal setpoint"
                    return self._terminal_event(
                        elapsed_s=elapsed_s,
                        duration_s=duration_s,
                        tick_lag_s=tick_lag_s,
                        lagged=lagged,
                    )

                timed_out = elapsed_s >= duration_s + self._policy.timeout_margin_s
                if timed_out:
                    detail = (
                        f"terminal feedback did not enter {self._policy.completion_tolerance_rad:.6f} rad tolerance "
                        f"within {self._policy.timeout_margin_s:.6f}s"
                    )
                    event_reason = MotionExecutionFailureReason.TIMEOUT
                    event_detail = detail
                    if self._apply_violation(
                        self._policy.timeout_action,
                        MotionExecutionFailureReason.TIMEOUT,
                        detail,
                        setpoint,
                    ):
                        return self._terminal_event(
                            elapsed_s=elapsed_s,
                            duration_s=duration_s,
                            tick_lag_s=tick_lag_s,
                            lagged=lagged,
                            timed_out=True,
                        )

            return MotionExecutionEvent(
                state=self._state,
                desired_setpoint=setpoint,
                elapsed_s=elapsed_s,
                duration_s=duration_s,
                progress=self._progress(elapsed_s, duration_s),
                tracking_error=tracking_error,
                max_tracking_error_rad=max_tracking_error_rad,
                tick_lag_s=tick_lag_s,
                lagged=lagged,
                timed_out=timed_out,
                reason=event_reason,
                detail=event_detail,
            )

    def cancel(
        self, hold_position: Optional[JointPositions] = None
    ) -> MotionExecutionResult:
        """Cancel trajectory progression without calling a physical stop API."""
        with self._lock:
            if self._state not in (
                MotionExecutionState.EXECUTING,
                MotionExecutionState.HOLDING,
                MotionExecutionState.CANCELED,
            ):
                return MotionExecutionResult(
                    accepted=False,
                    state=self._state,
                    reason=MotionExecutionFailureReason.INVALID_STATE,
                    detail=f"cannot cancel from {self._state.value}",
                )
            self._set_hold_setpoint(hold_position)
            self._state = MotionExecutionState.CANCELED
            self._reason = MotionExecutionFailureReason.CANCELED
            self._detail = "trajectory execution canceled"
            return MotionExecutionResult(
                accepted=True,
                state=self._state,
                reason=self._reason,
                detail=self._detail,
            )

    def hold(
        self, hold_position: Optional[JointPositions] = None
    ) -> MotionExecutionResult:
        """Freeze desired progression and expose a hold setpoint to the caller."""
        with self._lock:
            if self._state not in (
                MotionExecutionState.EXECUTING,
                MotionExecutionState.HOLDING,
            ):
                return MotionExecutionResult(
                    accepted=False,
                    state=self._state,
                    reason=MotionExecutionFailureReason.INVALID_STATE,
                    detail=f"cannot hold from {self._state.value}",
                )
            self._set_hold_setpoint(hold_position)
            self._state = MotionExecutionState.HOLDING
            self._reason = MotionExecutionFailureReason.HELD
            self._detail = "trajectory execution is holding"
            return MotionExecutionResult(
                accepted=True,
                state=self._state,
                reason=self._reason,
                detail=self._detail,
            )

    def fault(
        self,
        reason: MotionExecutionFailureReason,
        detail: str = "",
        hold_position: Optional[JointPositions] = None,
    ) -> MotionExecutionResult:
        """Record an externally-detected fault without touching hardware."""
        with self._lock:
            if not isinstance(reason, MotionExecutionFailureReason):
                raise TypeError("reason must be MotionExecutionFailureReason")
            self._set_hold_setpoint(hold_position)
            self._state = MotionExecutionState.FAULT
            self._reason = reason
            self._detail = str(detail)
            return MotionExecutionResult(
                accepted=True,
                state=self._state,
                reason=self._reason,
                detail=self._detail,
            )

    def reset(self) -> MotionExecutionResult:
        """Acknowledge a non-active state and clear the retained execution."""
        with self._lock:
            if self._state == MotionExecutionState.EXECUTING:
                return MotionExecutionResult(
                    accepted=False,
                    state=self._state,
                    reason=MotionExecutionFailureReason.EXECUTION_ACTIVE,
                    detail="cancel or hold the active trajectory before reset",
                )
            self._state = MotionExecutionState.IDLE
            self._trajectory = None
            self._started_at_s = None
            self._last_tick_at_s = None
            self._last_elapsed_s = None
            self._last_setpoint = None
            self._hold_setpoint = None
            self._last_tracking_error = None
            self._last_max_tracking_error_rad = None
            self._reason = None
            self._detail = ""
            return MotionExecutionResult(
                accepted=True,
                state=self._state,
                detail="executor reset",
            )

    def _terminal_event(
        self,
        elapsed_s: Optional[float] = None,
        duration_s: Optional[float] = None,
        tick_lag_s: Optional[float] = None,
        lagged: bool = False,
        timed_out: bool = False,
    ) -> MotionExecutionEvent:
        if elapsed_s is None:
            elapsed_s = self._last_elapsed_s
        if duration_s is None and self._trajectory is not None:
            duration_s = self._trajectory_duration_s(self._trajectory)
        desired = self._terminal_setpoint()
        return MotionExecutionEvent(
            state=self._state,
            desired_setpoint=desired,
            elapsed_s=elapsed_s,
            duration_s=duration_s,
            progress=self._progress(elapsed_s, duration_s),
            tracking_error=self._last_tracking_error,
            max_tracking_error_rad=self._last_max_tracking_error_rad,
            tick_lag_s=tick_lag_s,
            lagged=lagged,
            timed_out=timed_out,
            reason=self._reason,
            detail=self._detail,
        )

    def _terminal_setpoint(self) -> Optional[MotionExecutionSetpoint]:
        if self._state in (
            MotionExecutionState.HOLDING,
            MotionExecutionState.CANCELED,
            MotionExecutionState.FAULT,
        ):
            return self._hold_setpoint
        return self._last_setpoint

    def _transition_to_fault(
        self,
        reason: MotionExecutionFailureReason,
        detail: str,
        setpoint: Optional[MotionExecutionSetpoint],
    ) -> None:
        if setpoint is not None:
            self._hold_setpoint = self._as_hold_setpoint(setpoint)
        self._state = MotionExecutionState.FAULT
        self._reason = reason
        self._detail = detail

    def _apply_violation(
        self,
        action: MotionExecutionViolationAction,
        reason: MotionExecutionFailureReason,
        detail: str,
        setpoint: MotionExecutionSetpoint,
    ) -> bool:
        if action == MotionExecutionViolationAction.REPORT:
            return False
        self._hold_setpoint = self._as_hold_setpoint(setpoint)
        self._state = (
            MotionExecutionState.HOLDING
            if action == MotionExecutionViolationAction.HOLD
            else MotionExecutionState.FAULT
        )
        self._reason = reason
        self._detail = detail
        return True

    def _set_hold_setpoint(self, hold_position: Optional[JointPositions]) -> None:
        if hold_position is not None and not isinstance(hold_position, JointPositions):
            raise TypeError("hold_position must be JointPositions or None")
        if hold_position is not None:
            time_from_start_s = (
                self._last_setpoint.time_from_start_s
                if self._last_setpoint is not None
                else 0.0
            )
            self._hold_setpoint = MotionExecutionSetpoint(
                positions=hold_position,
                velocities=self._zero_joint_positions(),
                accelerations=self._zero_joint_positions(),
                time_from_start_s=time_from_start_s,
            )
        elif self._last_setpoint is not None:
            self._hold_setpoint = self._as_hold_setpoint(self._last_setpoint)

    @classmethod
    def _as_hold_setpoint(
        cls, setpoint: MotionExecutionSetpoint
    ) -> MotionExecutionSetpoint:
        return MotionExecutionSetpoint(
            positions=setpoint.positions,
            velocities=cls._zero_joint_positions(),
            accelerations=cls._zero_joint_positions(),
            time_from_start_s=setpoint.time_from_start_s,
        )

    def _tick_lag_s(self, now_s: float) -> Optional[float]:
        if self._last_tick_at_s is None:
            return None
        return max(0.0, now_s - self._last_tick_at_s)

    @classmethod
    def _tracking_error(
        cls,
        desired: JointPositions,
        actual: Optional[JointPositions],
    ) -> Tuple[Optional[JointPositions], Optional[float]]:
        if actual is None:
            return None, None
        values = tuple(
            desired_value - actual_value
            for desired_value, actual_value in zip(desired.values, actual.values)
        )
        return JointPositions(values), max(abs(value) for value in values)

    @classmethod
    def _sample(
        cls, trajectory: Any, time_from_start_s: float
    ) -> MotionExecutionSetpoint:
        sampled = sample_joint_trajectory(trajectory, time_from_start_s)
        return MotionExecutionSetpoint(
            positions=sampled.positions,
            velocities=sampled.velocities,
            accelerations=sampled.accelerations,
            time_from_start_s=sampled.time_from_start_s,
        )

    @classmethod
    def _setpoint_from_point(cls, point: Any) -> MotionExecutionSetpoint:
        return MotionExecutionSetpoint(
            positions=point.positions,
            velocities=cls._joint_positions_or_zero(point.velocities),
            accelerations=cls._joint_positions_or_zero(point.accelerations),
            time_from_start_s=point.time_from_start_s,
        )

    @classmethod
    def _validate_trajectory(cls, trajectory: Any) -> None:
        if trajectory is None:
            raise ValueError("trajectory must not be None")
        try:
            joint_names = tuple(trajectory.joint_names)
            points = tuple(trajectory.points)
        except (AttributeError, TypeError) as error:
            raise TypeError("trajectory must expose joint_names and points") from error
        if len(joint_names) != cls._JOINT_COUNT:
            raise ValueError("trajectory must contain exactly six joint names")
        if any(not isinstance(name, str) or not name.strip() for name in joint_names):
            raise ValueError("trajectory joint_names must be non-empty strings")
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("trajectory joint_names must be unique")
        if len(points) < 2:
            raise ValueError("trajectory must contain at least two points")

        previous_time_s: Optional[float] = None
        for index, point in enumerate(points):
            try:
                point_time_s = cls._finite_float(
                    point.time_from_start_s, "point time_from_start_s"
                )
                positions = point.positions
            except AttributeError as error:
                raise TypeError(
                    "trajectory point is missing required fields"
                ) from error
            if not isinstance(positions, JointPositions):
                raise TypeError("trajectory point positions must be JointPositions")
            cls._validate_optional_joint_positions(point.velocities, "velocities")
            cls._validate_optional_joint_positions(point.accelerations, "accelerations")
            if index == 0 and abs(point_time_s) > cls._TIME_EPSILON_S:
                raise ValueError("trajectory must start at time_from_start_s=0")
            if previous_time_s is not None and point_time_s <= previous_time_s:
                raise ValueError("trajectory timestamps must be strictly increasing")
            previous_time_s = point_time_s

    @classmethod
    def _trajectory_duration_s(cls, trajectory: Any) -> float:
        return cls._finite_float(
            tuple(trajectory.points)[-1].time_from_start_s,
            "trajectory duration",
        )

    @classmethod
    def _setpoint_from_optional_positions(
        cls, positions: Optional[JointPositions]
    ) -> JointPositions:
        return cls._joint_positions_or_zero(positions)

    @classmethod
    def _joint_positions_or_zero(
        cls, positions: Optional[JointPositions]
    ) -> JointPositions:
        if positions is None:
            return cls._zero_joint_positions()
        if not isinstance(positions, JointPositions):
            raise TypeError("trajectory derivative must be JointPositions or None")
        return positions

    @classmethod
    def _validate_optional_joint_positions(cls, value: Any, name: str) -> None:
        if value is not None and not isinstance(value, JointPositions):
            raise TypeError(f"trajectory point {name} must be JointPositions or None")

    @classmethod
    def _zero_joint_positions(cls) -> JointPositions:
        return JointPositions((0.0,) * cls._JOINT_COUNT)

    @staticmethod
    def _finite_float(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric, not boolean")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(number):
            raise ValueError(f"{name} must be finite")
        return number

    @classmethod
    def _normalize_monotonic_time(cls, value: Optional[float]) -> float:
        return cls._finite_float(
            time.monotonic() if value is None else value, "now_monotonic_s"
        )

    @staticmethod
    def _progress(elapsed_s: Optional[float], duration_s: Optional[float]) -> float:
        if elapsed_s is None or duration_s is None or duration_s <= 0.0:
            return 0.0
        return min(1.0, max(0.0, elapsed_s / duration_s))
