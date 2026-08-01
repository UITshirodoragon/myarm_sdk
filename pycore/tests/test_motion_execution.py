"""Focused tests for the transport-free motion-execution state machine."""

import pytest
from myarm_sdk.core import (
    JointPositions,
    JointTrajectory,
    TrajectoryPoint,
    load_sdk_yaml,
)
from myarm_sdk.core.motion_execution import (
    MotionExecutionFailureReason,
    MotionExecutionPolicy,
    MotionExecutionState,
    MotionExecutionViolationAction,
)
from myarm_sdk.plugin_adapter.motion_execution import (
    MonotonicTimeMotionExecutionAdapter,
)
from myarm_sdk.service.motion_execution import MotionExecutionService

JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_flex_joint",
    "forearm_roll_joint",
    "wrist_flex_joint",
    "wrist_roll_joint",
)


def _trajectory() -> JointTrajectory:
    zero = JointPositions((0.0,) * 6)
    one = JointPositions((1.0,) * 6)
    return JointTrajectory(
        joint_names=JOINT_NAMES,
        points=(
            TrajectoryPoint(
                positions=zero,
                velocities=zero,
                accelerations=zero,
                time_from_start_s=0.0,
            ),
            TrajectoryPoint(
                positions=one,
                velocities=zero,
                accelerations=zero,
                time_from_start_s=1.0,
            ),
        ),
    )


def test_samples_quintic_profile_and_succeeds_with_terminal_feedback() -> None:
    executor = MonotonicTimeMotionExecutionAdapter()
    started = executor.start(
        _trajectory(),
        policy=MotionExecutionPolicy(require_feedback_for_completion=True),
        now_monotonic_s=10.0,
    )

    assert started.accepted
    midpoint = executor.tick(
        now_monotonic_s=10.5,
        actual_positions=JointPositions((0.5,) * 6),
    )

    assert midpoint.state is MotionExecutionState.EXECUTING
    assert midpoint.desired_setpoint is not None
    assert midpoint.desired_setpoint.positions.values == pytest.approx((0.5,) * 6)
    assert midpoint.desired_setpoint.velocities.values == pytest.approx((1.875,) * 6)
    assert midpoint.desired_setpoint.accelerations.values == pytest.approx((0.0,) * 6)
    assert midpoint.tracking_error is not None
    assert midpoint.max_tracking_error_rad == pytest.approx(0.0)

    completed = executor.tick(
        now_monotonic_s=11.0,
        actual_positions=JointPositions((1.0,) * 6),
    )

    assert completed.state is MotionExecutionState.SUCCEEDED
    assert completed.progress == pytest.approx(1.0)


def test_rejects_concurrent_start_then_cancels_to_explicit_hold_setpoint() -> None:
    executor = MonotonicTimeMotionExecutionAdapter()
    assert executor.start(_trajectory(), now_monotonic_s=0.0).accepted

    rejected = executor.start(_trajectory(), now_monotonic_s=0.1)
    assert not rejected.accepted
    assert rejected.reason is MotionExecutionFailureReason.EXECUTION_ACTIVE

    executor.tick(now_monotonic_s=0.25)
    held_position = JointPositions((0.25,) * 6)
    canceled = executor.cancel(hold_position=held_position)
    event = executor.tick(now_monotonic_s=0.5)

    assert canceled.accepted
    assert event.state is MotionExecutionState.CANCELED
    assert event.desired_setpoint is not None
    assert event.desired_setpoint.positions == held_position
    assert event.desired_setpoint.velocities.values == (0.0,) * 6
    assert executor.reset().state is MotionExecutionState.IDLE


def test_reports_late_tick_then_holds_on_terminal_feedback_timeout() -> None:
    executor = MonotonicTimeMotionExecutionAdapter()
    policy = MotionExecutionPolicy(
        require_feedback_for_completion=True,
        completion_tolerance_rad=0.01,
        timeout_margin_s=0.2,
        max_tick_lag_s=0.1,
        tick_lag_action=MotionExecutionViolationAction.REPORT,
        timeout_action=MotionExecutionViolationAction.HOLD,
    )
    assert executor.start(_trajectory(), policy=policy, now_monotonic_s=0.0).accepted
    executor.tick(now_monotonic_s=0.0, actual_positions=JointPositions((0.0,) * 6))

    late = executor.tick(
        now_monotonic_s=0.5,
        actual_positions=JointPositions((0.0,) * 6),
    )
    timed_out = executor.tick(
        now_monotonic_s=1.3,
        actual_positions=JointPositions((0.0,) * 6),
    )

    assert late.state is MotionExecutionState.EXECUTING
    assert late.lagged
    assert late.reason is MotionExecutionFailureReason.TICK_LAG
    assert timed_out.state is MotionExecutionState.HOLDING
    assert timed_out.timed_out
    assert timed_out.reason is MotionExecutionFailureReason.TIMEOUT
    assert timed_out.desired_setpoint is not None
    assert timed_out.desired_setpoint.velocities.values == (0.0,) * 6


def test_service_loads_module_local_execution_profile() -> None:
    manifest = load_sdk_yaml("service/config/services.yaml")
    service = MotionExecutionService.from_config(
        manifest["services"]["motion_execution"]
    )

    assert service.settings.update_rate_hz == pytest.approx(5.0)
    assert service.settings.measured_state_max_age_s == pytest.approx(0.5)
    assert service.default_policy.require_feedback_for_completion
    assert service.default_policy.timeout_margin_s == pytest.approx(0.5)
    assert service.default_policy.max_tick_lag_s == pytest.approx(0.25)
