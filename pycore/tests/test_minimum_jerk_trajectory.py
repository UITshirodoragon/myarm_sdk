"""Focused behavior checks for the joint-space minimum-jerk planner."""

from __future__ import annotations

import math
from typing import Optional

import pytest
from myarm_sdk.core import (
    JointMetadata,
    JointMotionLimits,
    JointPositions,
    TimeScalingMode,
    TimeScalingPolicy,
    TrajectoryPlanningFailureReason,
    TrajectoryPlanningRequest,
)
from myarm_sdk.plugin_adapter.trajectory import MinimumJerkJointTrajectoryAdapter
from myarm_sdk.service import TrajectoryPlannerService


def _motion_limits() -> JointMotionLimits:
    metadata = tuple(
        JointMetadata(
            name=f"joint_{index + 1}",
            axis_xyz=(0.0, 0.0, 1.0),
            lower_limit_rad=-2.0,
            upper_limit_rad=2.0,
            velocity_limit_rad_s=1.0,
        )
        for index in range(6)
    )
    return JointMotionLimits(metadata, acceleration_limits_rad_s2=(1.0,) * 6)


def _request(
    policy: TimeScalingPolicy,
    target: Optional[JointPositions] = None,
) -> TrajectoryPlanningRequest:
    return TrajectoryPlanningRequest(
        q_start=JointPositions((0.0,) * 6),
        q_goal=target or JointPositions((1.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
        motion_limits=_motion_limits(),
        time_scaling=policy,
    )


def test_minimum_jerk_planner_outputs_complete_validated_trajectory():
    planner = MinimumJerkJointTrajectoryAdapter()
    result = planner.plan(_request(TimeScalingPolicy(sample_period_s=0.2)))

    assert result.succeeded
    assert result.trajectory is not None
    assert result.minimum_duration_s == pytest.approx(
        math.sqrt(10.0 / math.sqrt(3.0))
    )
    assert result.resolved_duration_s == pytest.approx(result.minimum_duration_s)
    assert result.trajectory.duration_s == result.resolved_duration_s
    assert result.trajectory.points[0].positions == JointPositions((0.0,) * 6)
    assert result.trajectory.points[-1].positions == JointPositions(
        (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
    assert result.trajectory.has_derivatives
    assert result.trajectory.points[0].velocities == JointPositions((0.0,) * 6)
    assert result.trajectory.points[-1].velocities == JointPositions((0.0,) * 6)
    assert result.trajectory.points[0].accelerations == JointPositions((0.0,) * 6)
    assert result.trajectory.points[-1].accelerations == JointPositions((0.0,) * 6)
    assert all(
        second.time_from_start_s > first.time_from_start_s
        for first, second in zip(
            result.trajectory.points, result.trajectory.points[1:]
        )
    )
    assert not _motion_limits().trajectory_violations(result.trajectory)


def test_requested_duration_stretch_extends_but_strict_rejects():
    planner = MinimumJerkJointTrajectoryAdapter()
    stretch_result = planner.plan(
        _request(
            TimeScalingPolicy(
                mode=TimeScalingMode.REQUESTED_DURATION_STRETCH,
                requested_duration_s=0.1,
            )
        )
    )
    strict_result = planner.plan(
        _request(
            TimeScalingPolicy(
                mode=TimeScalingMode.REQUESTED_DURATION_STRICT,
                requested_duration_s=0.1,
            )
        )
    )

    assert stretch_result.succeeded
    assert stretch_result.duration_adjusted
    assert stretch_result.resolved_duration_s == pytest.approx(
        stretch_result.minimum_duration_s
    )
    assert not strict_result.succeeded
    assert strict_result.trajectory is None
    assert (
        strict_result.failure_reason
        is TrajectoryPlanningFailureReason.DURATION_BELOW_LIMIT
    )


def test_speed_scale_stretches_the_resolved_base_duration():
    planner = MinimumJerkJointTrajectoryAdapter()
    automatic = planner.plan(_request(TimeScalingPolicy()))
    scaled = planner.plan(
        _request(
            TimeScalingPolicy(
                mode=TimeScalingMode.SPEED_SCALE,
                speed_scale=0.5,
            )
        )
    )

    assert automatic.succeeded and scaled.succeeded
    assert scaled.duration_adjusted
    assert scaled.resolved_duration_s == pytest.approx(
        automatic.resolved_duration_s / 0.5
    )


def test_out_of_limit_goal_returns_failure_without_trajectory():
    result = MinimumJerkJointTrajectoryAdapter().plan(
        _request(
            TimeScalingPolicy(),
            target=JointPositions((2.1, 0.0, 0.0, 0.0, 0.0, 0.0)),
        )
    )

    assert not result.succeeded
    assert result.trajectory is None
    assert result.failure_reason is TrajectoryPlanningFailureReason.GOAL_OUT_OF_LIMIT


def test_service_loads_the_named_acceleration_profile():
    metadata = tuple(
        JointMetadata(
            name=name,
            axis_xyz=(0.0, 0.0, 1.0),
            lower_limit_rad=-2.0,
            upper_limit_rad=2.0,
            velocity_limit_rad_s=1.0,
        )
        for name in (
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
        )
    )
    service = TrajectoryPlannerService.from_config(
        {
            "enabled": True,
            "plugin_adapter": "minimum_jerk_joint",
            "plugin_config": (
                "plugin_adapter/trajectory/config/"
                "minimum_jerk_joint_trajectory.yaml"
            ),
        },
        metadata,
    )

    assert service.motion_limits is not None
    assert service.motion_limits.acceleration_limits_rad_s2 == (0.5,) * 6
    assert service.default_time_scaling.mode is TimeScalingMode.REQUESTED_DURATION_STRETCH
