"""Shared joint-trajectory interpolation and dense limit validation.

The motion executor and Cartesian planner must evaluate the same polynomial.
Keeping the interpolation here prevents a Cartesian plan from passing sparse
waypoint checks while the executor later creates an unsafe in-between point.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Tuple

from .joint_positions import JointPositions
from .joint_trajectory import JointTrajectory
from .trajectory_point import TrajectoryPoint

_JOINT_COUNT = 6
_FLOAT_TOLERANCE = 1e-10


@dataclass(frozen=True)
class JointTrajectorySample:
    """One fully resolved trajectory sample in canonical joint order."""

    positions: JointPositions
    velocities: JointPositions
    accelerations: JointPositions
    time_from_start_s: float


def minimum_jerk_position_scale(normalized_time: float) -> float:
    """Return ``10u³ - 15u⁴ + 6u⁵`` clamped to the unit interval."""
    u = _unit_interval(normalized_time)
    squared = u * u
    cubed = squared * u
    return 10.0 * cubed - 15.0 * cubed * u + 6.0 * cubed * squared


def minimum_jerk_velocity_scale(normalized_time: float) -> float:
    """Return the derivative of :func:`minimum_jerk_position_scale`."""
    u = _unit_interval(normalized_time)
    return 30.0 * u * u * (1.0 - u) * (1.0 - u)


def minimum_jerk_acceleration_scale(normalized_time: float) -> float:
    """Return the second derivative of the minimum-jerk position scale."""
    u = _unit_interval(normalized_time)
    return 60.0 * u - 180.0 * u * u + 120.0 * u * u * u


def sample_joint_trajectory(
    trajectory: JointTrajectory, time_from_start_s: float
) -> JointTrajectorySample:
    """Sample a trajectory using the executor's linear/cubic/quintic rule."""
    sample_time_s = float(time_from_start_s)
    if not math.isfinite(sample_time_s):
        raise ValueError("time_from_start_s must be finite")
    points = trajectory.points
    if sample_time_s <= points[0].time_from_start_s:
        return _sample_from_point(points[0])
    if sample_time_s >= points[-1].time_from_start_s:
        return _sample_from_point(points[-1])
    for first, second in zip(points, points[1:]):
        if first.time_from_start_s <= sample_time_s <= second.time_from_start_s:
            return interpolate_joint_trajectory_segment(first, second, sample_time_s)
    raise RuntimeError("trajectory sampling failed to find a time segment")


def interpolate_joint_trajectory_segment(
    first: TrajectoryPoint,
    second: TrajectoryPoint,
    sample_time_s: float,
) -> JointTrajectorySample:
    """Interpolate one time segment with the executor-compatible polynomial."""
    if not first.time_from_start_s <= sample_time_s <= second.time_from_start_s:
        raise ValueError("sample_time_s must fall inside the supplied segment")
    duration_s = second.time_from_start_s - first.time_from_start_s
    if duration_s <= 0.0:
        raise ValueError("trajectory segment duration must be positive")
    local_time_s = sample_time_s - first.time_from_start_s
    if first.velocities is not None and second.velocities is not None:
        if first.accelerations is not None and second.accelerations is not None:
            values = _quintic_interpolation(first, second, local_time_s, duration_s)
        else:
            values = _cubic_interpolation(first, second, local_time_s, duration_s)
    else:
        values = _linear_interpolation(first, second, local_time_s, duration_s)
    return JointTrajectorySample(
        positions=JointPositions(values[0]),
        velocities=JointPositions(values[1]),
        accelerations=JointPositions(values[2]),
        time_from_start_s=float(sample_time_s),
    )


def dense_trajectory_violations(
    trajectory: JointTrajectory,
    motion_limits: Any,
    sample_period_s: float,
) -> Tuple[str, ...]:
    """Return hard limit violations at executor-interpolated samples.

    ``motion_limits`` intentionally uses a small duck-typed surface so this
    utility does not create a circular import with the planning value types.
    It must expose ``joint_names``, ``joint_metadata``,
    ``acceleration_limits_rad_s2`` and ``position_violations``.
    """
    sample_period_s = float(sample_period_s)
    if not math.isfinite(sample_period_s) or sample_period_s <= 0.0:
        raise ValueError("sample_period_s must be finite and positive")
    if trajectory.joint_names != tuple(motion_limits.joint_names):
        return ("trajectory joint_names do not match motion limits",)

    violations = []
    sampled_times = _dense_sample_times(trajectory, sample_period_s)
    for time_from_start_s in sampled_times:
        sample = sample_joint_trajectory(trajectory, time_from_start_s)
        label = f"t={time_from_start_s:.9f}s"
        for position_error in motion_limits.position_violations(sample.positions):
            violations.append(f"{label}: {position_error}")
        for index, metadata in enumerate(motion_limits.joint_metadata):
            velocity = abs(sample.velocities.values[index])
            if velocity > metadata.velocity_limit_rad_s + _FLOAT_TOLERANCE:
                violations.append(
                    f"{label}: {metadata.name} velocity {velocity:.9f} exceeds {metadata.velocity_limit_rad_s:.9f}"
                )
            acceleration = abs(sample.accelerations.values[index])
            acceleration_limit = motion_limits.acceleration_limits_rad_s2[index]
            if acceleration > acceleration_limit + _FLOAT_TOLERANCE:
                violations.append(
                    f"{label}: {metadata.name} acceleration {acceleration:.9f} exceeds {acceleration_limit:.9f}"
                )
    return tuple(violations)


def dense_sample_times(
    trajectory: JointTrajectory, sample_period_s: float
) -> Tuple[float, ...]:
    """Return monotonic endpoint-inclusive sample times for each segment."""
    return _dense_sample_times(trajectory, sample_period_s)


def _sample_from_point(point: TrajectoryPoint) -> JointTrajectorySample:
    return JointTrajectorySample(
        positions=point.positions,
        velocities=_joint_positions_or_zero(point.velocities),
        accelerations=_joint_positions_or_zero(point.accelerations),
        time_from_start_s=point.time_from_start_s,
    )


def _joint_positions_or_zero(value: Any) -> JointPositions:
    if value is None:
        return JointPositions((0.0,) * _JOINT_COUNT)
    if not isinstance(value, JointPositions):
        raise TypeError("trajectory derivative must be JointPositions or None")
    return value


def _dense_sample_times(
    trajectory: JointTrajectory, sample_period_s: float
) -> Tuple[float, ...]:
    step = float(sample_period_s)
    if not math.isfinite(step) or step <= 0.0:
        raise ValueError("sample_period_s must be finite and positive")
    times = [trajectory.points[0].time_from_start_s]
    for first, second in zip(trajectory.points, trajectory.points[1:]):
        duration_s = second.time_from_start_s - first.time_from_start_s
        intervals = max(1, math.ceil(duration_s / step))
        for index in range(1, intervals + 1):
            times.append(first.time_from_start_s + duration_s * index / intervals)
    return tuple(times)


def _linear_interpolation(
    first: TrajectoryPoint,
    second: TrajectoryPoint,
    local_time_s: float,
    duration_s: float,
) -> Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]]:
    ratio = local_time_s / duration_s
    positions = tuple(
        start + (target - start) * ratio
        for start, target in zip(first.positions.values, second.positions.values)
    )
    velocities = tuple(
        (target - start) / duration_s
        for start, target in zip(first.positions.values, second.positions.values)
    )
    return positions, velocities, (0.0,) * _JOINT_COUNT


def _cubic_interpolation(
    first: TrajectoryPoint,
    second: TrajectoryPoint,
    local_time_s: float,
    duration_s: float,
) -> Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]]:
    if first.velocities is None or second.velocities is None:
        raise ValueError("cubic interpolation requires endpoint velocities")
    u = local_time_s / duration_s
    u2 = u * u
    h00 = 2.0 * u2 * u - 3.0 * u2 + 1.0
    h10 = u2 * u - 2.0 * u2 + u
    h01 = -2.0 * u2 * u + 3.0 * u2
    h11 = u2 * u - u2
    h00_dot = 6.0 * u2 - 6.0 * u
    h10_dot = 3.0 * u2 - 4.0 * u + 1.0
    h01_dot = -6.0 * u2 + 6.0 * u
    h11_dot = 3.0 * u2 - 2.0 * u
    h00_ddot = 12.0 * u - 6.0
    h10_ddot = 6.0 * u - 4.0
    h01_ddot = -12.0 * u + 6.0
    h11_ddot = 6.0 * u - 2.0
    positions = []
    velocities = []
    accelerations = []
    for q0, q1, v0, v1 in zip(
        first.positions.values,
        second.positions.values,
        first.velocities.values,
        second.velocities.values,
    ):
        positions.append(
            h00 * q0 + h10 * duration_s * v0 + h01 * q1 + h11 * duration_s * v1
        )
        velocities.append(
            (
                h00_dot * q0
                + h10_dot * duration_s * v0
                + h01_dot * q1
                + h11_dot * duration_s * v1
            )
            / duration_s
        )
        accelerations.append(
            (
                h00_ddot * q0
                + h10_ddot * duration_s * v0
                + h01_ddot * q1
                + h11_ddot * duration_s * v1
            )
            / (duration_s * duration_s)
        )
    return tuple(positions), tuple(velocities), tuple(accelerations)


def _quintic_interpolation(
    first: TrajectoryPoint,
    second: TrajectoryPoint,
    local_time_s: float,
    duration_s: float,
) -> Tuple[Tuple[float, ...], Tuple[float, ...], Tuple[float, ...]]:
    if (
        first.velocities is None
        or second.velocities is None
        or first.accelerations is None
        or second.accelerations is None
    ):
        raise ValueError("quintic interpolation requires endpoint derivatives")
    positions = []
    velocities = []
    accelerations = []
    t = local_time_s
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t
    duration2 = duration_s * duration_s
    duration3 = duration2 * duration_s
    duration4 = duration3 * duration_s
    duration5 = duration4 * duration_s
    for q0, q1, v0, v1, a0, a1 in zip(
        first.positions.values,
        second.positions.values,
        first.velocities.values,
        second.velocities.values,
        first.accelerations.values,
        second.accelerations.values,
    ):
        delta_q = q1 - (q0 + v0 * duration_s + 0.5 * a0 * duration2)
        delta_v = v1 - (v0 + a0 * duration_s)
        delta_a = a1 - a0
        c0 = q0
        c1 = v0
        c2 = 0.5 * a0
        c3 = (
            10.0 * delta_q
            - 4.0 * delta_v * duration_s
            + 0.5 * delta_a * duration2
        ) / duration3
        c4 = (
            -15.0 * delta_q
            + 7.0 * delta_v * duration_s
            - delta_a * duration2
        ) / duration4
        c5 = (
            6.0 * delta_q
            - 3.0 * delta_v * duration_s
            + 0.5 * delta_a * duration2
        ) / duration5
        positions.append(c0 + c1 * t + c2 * t2 + c3 * t3 + c4 * t4 + c5 * t5)
        velocities.append(
            c1 + 2.0 * c2 * t + 3.0 * c3 * t2 + 4.0 * c4 * t3 + 5.0 * c5 * t4
        )
        accelerations.append(
            2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t2 + 20.0 * c5 * t3
        )
    return tuple(positions), tuple(velocities), tuple(accelerations)


def _unit_interval(value: float) -> float:
    normalized = float(value)
    if not math.isfinite(normalized):
        raise ValueError("normalized_time must be finite")
    return max(0.0, min(1.0, normalized))
