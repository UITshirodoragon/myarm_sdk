"""Application-facing service for transport-free trajectory execution."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Mapping, Optional

from myarm_sdk.core.configuration import load_sdk_yaml
from myarm_sdk.core.joint_positions import JointPositions
from myarm_sdk.core.motion_execution import (
    MotionExecutionEvent,
    MotionExecutionFailureReason,
    MotionExecutionPolicy,
    MotionExecutionResult,
    MotionExecutionState,
    MotionExecutionViolationAction,
)
from myarm_sdk.core.validation import require_enabled
from myarm_sdk.plugin_adapter.motion_execution import (
    MonotonicTimeMotionExecutionAdapter,
)
from myarm_sdk.port_interface.motion_execution import MotionExecutionInterface

if TYPE_CHECKING:
    from myarm_sdk.core.joint_trajectory import JointTrajectory


class MotionExecutionServiceError(RuntimeError):
    """A motion-execution service configuration or policy was invalid."""


@dataclass(frozen=True)
class MotionExecutionServiceSettings:
    """ROS-boundary settings owned by the motion-execution service manifest."""

    update_rate_hz: float
    measured_state_max_age_s: float
    start_tolerance_rad: float


class MotionExecutionService:
    """Own a motion-execution port without owning a robot-arm connection.

    The ROS action node uses this service to progress an active trajectory.
    The node, not this class, publishes the returned desired
    setpoint to the independent robot-driver transport boundary.
    """

    def __init__(
        self,
        motion_execution: MotionExecutionInterface,
        default_policy: Optional[MotionExecutionPolicy] = None,
        settings: Optional[MotionExecutionServiceSettings] = None,
    ) -> None:
        self._motion_execution = motion_execution
        self._default_policy = default_policy or MotionExecutionPolicy()
        self._settings = settings or MotionExecutionServiceSettings(
            update_rate_hz=5.0,
            measured_state_max_age_s=0.5,
            start_tolerance_rad=0.05,
        )

    @classmethod
    def from_config(cls, service_config: Mapping[str, Any]) -> MotionExecutionService:
        """Construct the configured pure executor and its policy.

        This factory intentionally loads only the module-local plugin profile
        and the ``services.motion_execution`` block. It does not create a
        robot adapter or import ROS.
        """
        require_enabled(service_config, "motion_execution")
        if service_config.get("plugin_adapter") != "monotonic_time_motion_execution":
            raise MotionExecutionServiceError(
                "Only the monotonic_time_motion_execution plugin adapter is available"
            )
        plugin_config_path = cls._required_string(
            service_config.get("plugin_config"), "motion_execution plugin_config"
        )
        plugin_config = load_sdk_yaml(plugin_config_path)
        if plugin_config.get("plugin_adapter") != "monotonic_time_motion_execution":
            raise MotionExecutionServiceError(
                "Motion-execution plugin config must select "
                "monotonic_time_motion_execution"
            )

        policy_config = cls._mapping(
            plugin_config.get("policy"), "motion_execution plugin policy"
        )
        feedback_config = cls._mapping(
            service_config.get("feedback"), "motion_execution feedback"
        )
        execution_config = cls._mapping(
            service_config.get("execution"), "motion_execution execution"
        )
        policy = MotionExecutionPolicy(
            completion_tolerance_rad=cls._non_negative_finite(
                policy_config.get("completion_tolerance_rad"),
                "motion_execution policy.completion_tolerance_rad",
            ),
            require_feedback_for_completion=cls._boolean(
                policy_config.get("require_feedback_for_completion", False),
                "motion_execution policy.require_feedback_for_completion",
            ),
            timeout_margin_s=cls._non_negative_finite(
                execution_config.get("completion_timeout_margin_s"),
                "motion_execution execution.completion_timeout_margin_s",
            ),
            max_tick_lag_s=cls._optional_positive_finite(
                execution_config.get("late_tick_tolerance_s"),
                "motion_execution execution.late_tick_tolerance_s",
            ),
            tick_lag_action=cls._violation_action(
                policy_config.get("tick_lag_action"),
                "motion_execution policy.tick_lag_action",
            ),
            max_tracking_error_rad=cls._optional_positive_finite(
                policy_config.get("max_tracking_error_rad"),
                "motion_execution policy.max_tracking_error_rad",
            ),
            tracking_error_action=cls._violation_action(
                policy_config.get("tracking_error_action"),
                "motion_execution policy.tracking_error_action",
            ),
            timeout_action=cls._violation_action(
                policy_config.get("timeout_action"),
                "motion_execution policy.timeout_action",
            ),
        )
        settings = MotionExecutionServiceSettings(
            update_rate_hz=cls._positive_finite(
                service_config.get("update_rate_hz"),
                "motion_execution update_rate_hz",
            ),
            measured_state_max_age_s=cls._positive_finite(
                feedback_config.get("measured_state_max_age_s"),
                "motion_execution feedback.measured_state_max_age_s",
            ),
            start_tolerance_rad=cls._non_negative_finite(
                feedback_config.get("start_tolerance_rad"),
                "motion_execution feedback.start_tolerance_rad",
            ),
        )
        return cls(
            motion_execution=MonotonicTimeMotionExecutionAdapter(),
            default_policy=policy,
            settings=settings,
        )

    @property
    def state(self) -> MotionExecutionState:
        return self._motion_execution.state

    @property
    def default_policy(self) -> MotionExecutionPolicy:
        """Return the immutable policy selected by the module configuration."""
        return self._default_policy

    @property
    def settings(self) -> MotionExecutionServiceSettings:
        """Return node-facing rate and feedback freshness settings."""
        return self._settings

    def start(
        self,
        trajectory: JointTrajectory,
        policy: Optional[MotionExecutionPolicy] = None,
        now_monotonic_s: Optional[float] = None,
    ) -> MotionExecutionResult:
        """Accept a validated trajectory without blocking or touching hardware."""
        return self._motion_execution.start(
            trajectory=trajectory,
            policy=policy or self._default_policy,
            now_monotonic_s=now_monotonic_s,
        )

    def tick(
        self,
        now_monotonic_s: Optional[float] = None,
        actual_positions: Optional[JointPositions] = None,
    ) -> MotionExecutionEvent:
        """Sample desired state and optional measured tracking error."""
        return self._motion_execution.tick(
            now_monotonic_s=now_monotonic_s,
            actual_positions=actual_positions,
        )

    def cancel(
        self, hold_position: Optional[JointPositions] = None
    ) -> MotionExecutionResult:
        """Cancel execution; forwarding the hold setpoint is a caller decision."""
        return self._motion_execution.cancel(hold_position=hold_position)

    def hold(
        self, hold_position: Optional[JointPositions] = None
    ) -> MotionExecutionResult:
        """Freeze timing and retain a setpoint without issuing hardware I/O."""
        return self._motion_execution.hold(hold_position=hold_position)

    def fault(
        self,
        reason: MotionExecutionFailureReason,
        detail: str = "",
        hold_position: Optional[JointPositions] = None,
    ) -> MotionExecutionResult:
        """Record an external fault at the execution boundary."""
        return self._motion_execution.fault(
            reason=reason,
            detail=detail,
            hold_position=hold_position,
        )

    def reset(self) -> MotionExecutionResult:
        """Acknowledge a terminal state before accepting another trajectory."""
        return self._motion_execution.reset()

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise MotionExecutionServiceError(f"{name} must be a mapping")
        return value

    @staticmethod
    def _required_string(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise MotionExecutionServiceError(f"{name} must be a non-empty string")
        return value

    @staticmethod
    def _positive_finite(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise MotionExecutionServiceError(f"{name} must be numeric, not boolean")
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise MotionExecutionServiceError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise MotionExecutionServiceError(f"{name} must be finite and positive")
        return normalized

    @staticmethod
    def _boolean(value: Any, name: str) -> bool:
        if not isinstance(value, bool):
            raise MotionExecutionServiceError(f"{name} must be boolean")
        return value

    @classmethod
    def _non_negative_finite(cls, value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise MotionExecutionServiceError(f"{name} must be numeric, not boolean")
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise MotionExecutionServiceError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or normalized < 0.0:
            raise MotionExecutionServiceError(f"{name} must be finite and non-negative")
        return normalized

    @classmethod
    def _optional_positive_finite(cls, value: Any, name: str) -> Optional[float]:
        if value is None:
            return None
        return cls._positive_finite(value, name)

    @staticmethod
    def _violation_action(value: Any, name: str) -> MotionExecutionViolationAction:
        try:
            return MotionExecutionViolationAction(str(value))
        except ValueError as error:
            supported = ", ".join(
                action.value for action in MotionExecutionViolationAction
            )
            raise MotionExecutionServiceError(
                f"{name} must be one of: {supported}"
            ) from error
