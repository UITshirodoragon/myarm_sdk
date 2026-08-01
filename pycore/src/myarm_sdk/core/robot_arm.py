"""ROS-independent state and errors for a canonical robot-arm backend."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Optional

from .joint_positions import JointPositions


class RobotArmError(RuntimeError):
    """Base error raised by a robot-arm implementation."""


class RobotArmConnectionError(RobotArmError):
    """A robot-arm backend could not be connected or disconnected safely."""


class RobotArmLifecycleError(RobotArmError):
    """An operation was requested while the robot is not ready for it."""


class RobotArmLimitError(RobotArmError):
    """A measured or commanded joint position violates configured limits."""


class RobotArmProtocolError(RobotArmError):
    """A robot-arm backend returned malformed data or an explicit error."""


@dataclass(frozen=True)
class GripperCommand:
    """One accepted parallel-gripper command in total fingertip opening metres."""

    requested_opening_width_m: float
    accepted_opening_width_m: float
    speed_scale: float
    issued_at_monotonic_s: float = field(default_factory=time.monotonic)
    sequence: int = 0

    MAX_OPENING_WIDTH_M = 0.08

    def __post_init__(self) -> None:
        for name in (
            "requested_opening_width_m",
            "accepted_opening_width_m",
            "speed_scale",
            "issued_at_monotonic_s",
        ):
            value = getattr(self, name)
            if isinstance(value, bool):
                raise TypeError(f"{name} must be numeric, not boolean")
            normalized = float(value)
            if not math.isfinite(normalized):
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, normalized)
        for name in ("requested_opening_width_m", "accepted_opening_width_m"):
            value = getattr(self, name)
            if not 0.0 <= value <= self.MAX_OPENING_WIDTH_M:
                raise ValueError(
                    f"{name} must be in [0, {self.MAX_OPENING_WIDTH_M}] metres"
                )
        if not 0.0 < self.speed_scale <= 1.0:
            raise ValueError("speed_scale must be finite and in the range (0, 1]")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise TypeError("sequence must be an integer")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")


@dataclass(frozen=True)
class GripperState:
    """Cached gripper feedback in total fingertip opening metres.

    ``opening_width_m`` is the distance between both fingertips, not the URDF
    coordinate of one jaw.  The latter is always ``opening_width_m / 2``.
    Physical adapters keep it ``None`` until a vendor feedback read succeeds.
    """

    opening_width_m: Optional[float] = None
    raw_vendor_value: Optional[int] = None
    is_enabled: Optional[bool] = None
    is_moving: Optional[bool] = None
    measured_at_monotonic_s: Optional[float] = None
    last_command: Optional[GripperCommand] = None
    sequence: int = 0
    consecutive_error_count: int = 0
    last_error_message: Optional[str] = None

    def __post_init__(self) -> None:
        if self.opening_width_m is not None:
            if isinstance(self.opening_width_m, bool):
                raise TypeError("opening_width_m must be numeric or None")
            opening_width_m = float(self.opening_width_m)
            if (
                not math.isfinite(opening_width_m)
                or not 0.0 <= opening_width_m <= GripperCommand.MAX_OPENING_WIDTH_M
            ):
                raise ValueError("opening_width_m must be in [0, 0.08] metres")
            object.__setattr__(self, "opening_width_m", opening_width_m)
        has_opening = self.opening_width_m is not None
        has_timestamp = self.measured_at_monotonic_s is not None
        if has_opening != has_timestamp:
            raise ValueError(
                "opening_width_m and measured_at_monotonic_s must be set together"
            )
        if has_timestamp and not math.isfinite(self.measured_at_monotonic_s):
            raise ValueError("measured_at_monotonic_s must be finite")
        if self.raw_vendor_value is not None:
            if isinstance(self.raw_vendor_value, bool) or not isinstance(
                self.raw_vendor_value, int
            ):
                raise TypeError("raw_vendor_value must be an integer or None")
            if not 0 <= self.raw_vendor_value <= 100:
                raise ValueError("raw_vendor_value must be in the range 0..100")
        for name in ("is_enabled", "is_moving"):
            value = getattr(self, name)
            if value is not None and not isinstance(value, bool):
                raise TypeError(f"{name} must be boolean or None")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise TypeError("sequence must be an integer")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        if (
            isinstance(self.consecutive_error_count, bool)
            or not isinstance(self.consecutive_error_count, int)
            or self.consecutive_error_count < 0
        ):
            raise ValueError("consecutive_error_count must be a non-negative integer")
        if self.last_error_message is not None and (
            not isinstance(self.last_error_message, str)
            or not self.last_error_message.strip()
        ):
            raise ValueError("last_error_message must be a non-empty string or None")


@dataclass(frozen=True)
class RobotArmCommand:
    """One accepted canonical joint-position command.

    ``requested_joint_positions`` preserves the caller input.  For physical
    hardware, ``accepted_joint_positions`` records the command after the
    backend's representable-value quantization and calibration mapping.
    """

    requested_joint_positions: JointPositions
    accepted_joint_positions: JointPositions
    speed_scale: float
    issued_at_monotonic_s: float = field(default_factory=time.monotonic)
    sequence: int = 0

    def __post_init__(self) -> None:
        if isinstance(self.speed_scale, bool):
            raise TypeError("speed_scale must be numeric, not boolean")
        speed_scale = float(self.speed_scale)
        if not math.isfinite(speed_scale) or not 0.0 < speed_scale <= 1.0:
            raise ValueError("speed_scale must be finite and in the range (0, 1]")
        if not math.isfinite(self.issued_at_monotonic_s):
            raise ValueError("issued_at_monotonic_s must be finite")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int):
            raise TypeError("sequence must be an integer")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        object.__setattr__(self, "speed_scale", speed_scale)


@dataclass(frozen=True)
class RobotArmState:
    """Immutable cached state owned by a robot-arm implementation.

    ``measured_joint_positions`` only changes after a feedback read (or an
    immediate fake execution).  A successful physical command updates only
    ``last_command`` until the next measurement confirms where the arm is.
    """

    source: str = "unknown"
    is_connected: bool = False
    is_powered: Optional[bool] = None
    is_moving: Optional[bool] = None
    measured_joint_positions: Optional[JointPositions] = None
    measured_at_monotonic_s: Optional[float] = None
    last_command: Optional[RobotArmCommand] = None
    gripper_state: Optional[GripperState] = None
    sequence: int = 0
    consecutive_error_count: int = 0
    last_error_message: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.source, str):
            raise TypeError("source must be a string")
        if not self.source.strip():
            raise ValueError("source must be a non-empty string")
        if not isinstance(self.is_connected, bool):
            raise TypeError("is_connected must be boolean")
        for field_name, value in (
            ("is_powered", self.is_powered),
            ("is_moving", self.is_moving),
        ):
            if value is not None and not isinstance(value, bool):
                raise TypeError(f"{field_name} must be boolean or None")
        has_positions = self.measured_joint_positions is not None
        has_timestamp = self.measured_at_monotonic_s is not None
        if has_positions != has_timestamp:
            raise ValueError(
                "measured_joint_positions and measured_at_monotonic_s must be set together"
            )
        if has_timestamp and not math.isfinite(self.measured_at_monotonic_s):
            raise ValueError("measured_at_monotonic_s must be finite")
        if self.gripper_state is not None and not isinstance(
            self.gripper_state, GripperState
        ):
            raise TypeError("gripper_state must be GripperState or None")
        if (
            isinstance(self.sequence, bool)
            or not isinstance(self.sequence, int)
        ):
            raise TypeError("sequence must be an integer")
        if self.sequence < 0:
            raise ValueError("sequence must be non-negative")
        if (
            isinstance(self.consecutive_error_count, bool)
            or not isinstance(self.consecutive_error_count, int)
        ):
            raise TypeError("consecutive_error_count must be an integer")
        if self.consecutive_error_count < 0:
            raise ValueError("consecutive_error_count must be non-negative")
        if self.last_error_message is not None and (
            not isinstance(self.last_error_message, str)
            or not self.last_error_message.strip()
        ):
            raise ValueError("last_error_message must be a non-empty string or None")
