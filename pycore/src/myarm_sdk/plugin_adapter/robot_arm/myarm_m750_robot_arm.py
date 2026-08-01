"""Stateful ``MyArmMControl`` implementation for a physical MyArm M750."""

from __future__ import annotations

import importlib
import math
import threading
import time
from dataclasses import replace
from typing import Any, Callable, Optional, Sequence, Tuple

from myarm_sdk.core import (
    JointMetadata,
    JointPositions,
    RobotArmCommand,
    RobotArmConnectionError,
    RobotArmLifecycleError,
    RobotArmLimitError,
    RobotArmProtocolError,
    RobotArmState,
)


def _load_myarm_mcontrol_factory() -> Callable[..., Any]:
    """Import the vendor class only when a physical connection is requested."""
    attempts = (
        ("pymycobot.myarmm_control", "MyArmMControl"),
        ("pymycobot", "MyArmMControl"),
    )
    errors = []
    for module_name, class_name in attempts:
        try:
            module = importlib.import_module(module_name)
            return getattr(module, class_name)
        except (ImportError, AttributeError) as error:
            errors.append(f"{module_name}: {error}")
    raise RobotArmConnectionError(
        "Unable to import pymycobot MyArmMControl. Install robot support with "
        "`pip install myarm-sdk[robot-arm]`. Attempts: {}".format(
            "; ".join(errors)
        )
    )


class MyArmM750RobotArm:
    """Translate canonical M750 joints to the blocking ``MyArmMControl`` API.

    The adapter owns one vendor SDK object and serializes an entire vendor
    request/response transaction with ``RLock``.  Public positions are always
    canonical URDF radians.  A command acknowledgement changes only
    ``state.last_command``; ``state.measured_joint_positions`` is changed only
    after ``read_state`` receives feedback from the physical arm.

    ``joint_metadata`` is deliberately mandatory: the caller must supply the
    canonical six-joint metadata loaded from the same URDF used by kinematics.
    This prevents a physical command path without authoritative joint limits.
    """

    DEFAULT_JOINT_NAMES = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    )
    DEFAULT_BAUDRATE = 1_000_000
    DEFAULT_TIMEOUT_S = 0.1
    DEFAULT_SPEED_SCALE = 0.5
    DEFAULT_FRESH_MODE = 1
    DEFAULT_MODEL_TO_HARDWARE_OFFSETS_RAD = (
        0.0,
        math.radians(10.0),
        math.radians(-10.0),
        0.0,
        0.0,
        0.0,
    )

    def __init__(
        self,
        serial_port: str,
        joint_metadata: Sequence[JointMetadata],
        baudrate: int = DEFAULT_BAUDRATE,
        timeout_s: float = DEFAULT_TIMEOUT_S,
        debug: bool = False,
        model_to_hardware_offsets_rad: Sequence[float] = (
            DEFAULT_MODEL_TO_HARDWARE_OFFSETS_RAD
        ),
        vendor_factory: Optional[Callable[..., Any]] = None,
    ) -> None:
        if not isinstance(serial_port, str) or not serial_port.strip():
            raise ValueError("serial_port must be a non-empty string")
        if isinstance(baudrate, bool) or not isinstance(baudrate, int) or baudrate <= 0:
            raise ValueError("baudrate must be a positive integer")
        if isinstance(timeout_s, bool):
            raise TypeError("timeout_s must be numeric, not boolean")
        try:
            timeout = float(timeout_s)
        except (TypeError, ValueError) as error:
            raise TypeError("timeout_s must be numeric") from error
        if not math.isfinite(timeout):
            raise ValueError("timeout_s must be finite")
        if timeout <= 0.0:
            raise ValueError("timeout_s must be positive")
        if not isinstance(debug, bool):
            raise TypeError("debug must be boolean")
        if vendor_factory is not None and not callable(vendor_factory):
            raise TypeError("vendor_factory must be callable or None")

        offsets = tuple(float(value) for value in model_to_hardware_offsets_rad)
        if len(offsets) != len(self.DEFAULT_JOINT_NAMES) or not all(
            math.isfinite(value) for value in offsets
        ):
            raise ValueError(
                "model_to_hardware_offsets_rad requires six finite values"
            )

        self._serial_port = serial_port
        self._baudrate = baudrate
        self._timeout_s = timeout
        self._debug = debug
        self._model_to_hardware_offsets_rad = offsets
        self._joint_metadata = self._validate_joint_metadata(joint_metadata)
        self._vendor_factory = vendor_factory
        self._vendor: Optional[Any] = None
        self._state = RobotArmState(source="myarm_m750_robot_arm")
        self._lock = threading.RLock()

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return self.DEFAULT_JOINT_NAMES

    @property
    def state(self) -> RobotArmState:
        with self._lock:
            return self._state

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._vendor is not None

    def connect(self) -> RobotArmState:
        """Open serial, configure latest-command mode, and read power state."""
        with self._lock:
            if self._vendor is not None:
                return self._state
            factory = self._vendor_factory or _load_myarm_mcontrol_factory()
            try:
                vendor = factory(
                    self._serial_port,
                    baudrate=self._baudrate,
                    timeout=self._timeout_s,
                    debug=self._debug,
                )
            except Exception as error:  # Vendor/serial error is normalized here.
                self._record_error(f"failed to open serial connection: {error}")
                raise RobotArmConnectionError(
                    f"Unable to open MyArm M750 serial port {self._serial_port}: {error}"
                ) from error

            self._vendor = vendor
            self._replace_state(is_connected=True, is_powered=None, is_moving=None)
            try:
                self._call_vendor(
                    "set_fresh_mode",
                    lambda: vendor.set_fresh_mode(self.DEFAULT_FRESH_MODE),
                )
                fresh_mode = self._call_vendor(
                    "get_fresh_mode", vendor.get_fresh_mode
                )
                if fresh_mode != self.DEFAULT_FRESH_MODE:
                    message = (
                        f"MyArmMControl fresh mode must be {self.DEFAULT_FRESH_MODE}, "
                        f"got {fresh_mode!r}"
                    )
                    self._record_error(message)
                    raise RobotArmProtocolError(message)
                self._read_power_state_locked()
                return self.read_state()
            except Exception:
                self._disconnect_vendor_after_failed_connect(vendor)
                raise

    def disconnect(self) -> RobotArmState:
        """Close serial transport without sending an implicit power-off command."""
        with self._lock:
            vendor = self._vendor
            if vendor is None:
                return self._state
            try:
                self._close_vendor_transport(vendor)
            except Exception as error:  # Preserve an actionable lifecycle error.
                self._record_error(f"failed to close serial connection: {error}")
                raise RobotArmConnectionError(
                    f"Unable to close MyArm M750 serial port {self._serial_port}: {error}"
                ) from error
            self._vendor = None
            return self._replace_state(is_connected=False, is_moving=None)

    def read_state(self) -> RobotArmState:
        """Read six feedback angles and cache canonical model-space positions."""
        with self._lock:
            vendor = self._require_vendor()
            response = self._call_vendor("get_angles", vendor.get_angles)
            try:
                hardware_angle_deg = self._validate_hardware_angles(response)
                hardware_positions_rad = tuple(
                    math.radians(value) for value in hardware_angle_deg
                )
                measured_joint_positions = self.model_from_hardware_positions(
                    hardware_positions_rad
                )
                self._validate_joint_limits(measured_joint_positions)
            except (RobotArmProtocolError, RobotArmLimitError) as error:
                self._record_error(str(error))
                raise
            return self._replace_state(
                measured_joint_positions=measured_joint_positions,
                measured_at_monotonic_s=time.monotonic(),
            )

    def write_joint_positions(
        self, target: JointPositions, speed_scale: float = DEFAULT_SPEED_SCALE
    ) -> RobotArmCommand:
        """Write one six-joint position target through ``write_angles``."""
        with self._lock:
            vendor = self._require_vendor()
            self._require_powered_for_motion()
            self._require_initial_measurement()
            vendor_speed = self._vendor_speed_from_scale(speed_scale)
            self._validate_joint_limits(target)

            hardware_positions_rad = self.hardware_from_model_positions(target)
            hardware_angle_deg = tuple(
                math.degrees(value) for value in hardware_positions_rad
            )
            accepted_hardware_angle_deg = tuple(
                self._quantize_hardware_angle_deg(value)
                for value in hardware_angle_deg
            )
            accepted_joint_positions = self.model_from_hardware_positions(
                tuple(math.radians(value) for value in accepted_hardware_angle_deg)
            )
            self._validate_joint_limits(accepted_joint_positions)

            self._call_vendor(
                "write_angles",
                lambda: vendor.write_angles(list(hardware_angle_deg), vendor_speed),
            )
            now_s = time.monotonic()
            next_sequence = self._state.sequence + 1
            command = RobotArmCommand(
                requested_joint_positions=target,
                accepted_joint_positions=accepted_joint_positions,
                speed_scale=speed_scale,
                issued_at_monotonic_s=now_s,
                sequence=next_sequence,
            )
            self._state = replace(
                self._state,
                last_command=command,
                is_moving=None,
                sequence=next_sequence,
                consecutive_error_count=0,
                last_error_message=None,
            )
            return command

    def stop(self) -> RobotArmState:
        """Request the firmware software stop; this is not an emergency stop."""
        with self._lock:
            vendor = self._require_vendor()
            self._call_vendor("stop", vendor.stop)
            return self._replace_state(is_moving=None)

    def power_on(self) -> RobotArmState:
        with self._lock:
            vendor = self._require_vendor()
            self._call_vendor("power_on", vendor.power_on)
            return self._read_power_state_locked(expected_powered=True)

    def power_off(self) -> RobotArmState:
        with self._lock:
            vendor = self._require_vendor()
            self._call_vendor("power_off", vendor.power_off)
            return self._read_power_state_locked(expected_powered=False)

    def read_power_state(self) -> RobotArmState:
        with self._lock:
            return self._read_power_state_locked()

    def read_motion_state(self) -> RobotArmState:
        with self._lock:
            vendor = self._require_vendor()
            is_moving = self._read_vendor_boolean("is_moving", vendor.is_moving)
            return self._replace_state(is_moving=is_moving)

    # Compatibility wrappers retained while callers move to the stateful API.
    def read_joints(self) -> JointPositions:
        state = self.read_state()
        assert state.measured_joint_positions is not None
        return state.measured_joint_positions

    def move_joints(self, target: JointPositions, speed: int = 50) -> None:
        if isinstance(speed, bool) or not isinstance(speed, int) or not 1 <= speed <= 100:
            raise ValueError("speed must be an integer in the range 1..100")
        self.write_joint_positions(target, speed_scale=float(speed) / 100.0)

    def close(self) -> None:
        self.disconnect()

    def model_from_hardware_positions(
        self, hardware_positions_rad: Sequence[float]
    ) -> JointPositions:
        """Convert feedback ``q_hardware`` into canonical URDF ``q_model``."""
        values = self._six_finite_values(
            hardware_positions_rad, "hardware_positions_rad"
        )
        return JointPositions(
            tuple(
                value - offset
                for value, offset in zip(values, self._model_to_hardware_offsets_rad)
            )
        )

    def hardware_from_model_positions(
        self, model_positions: JointPositions
    ) -> Tuple[float, ...]:
        """Convert canonical URDF ``q_model`` into firmware ``q_hardware``."""
        return tuple(
            value + offset
            for value, offset in zip(
                model_positions.values, self._model_to_hardware_offsets_rad
            )
        )

    @classmethod
    def _validate_joint_metadata(
        cls, joint_metadata: Sequence[JointMetadata]
    ) -> Tuple[JointMetadata, ...]:
        metadata = tuple(joint_metadata)
        if len(metadata) != len(cls.DEFAULT_JOINT_NAMES):
            raise ValueError("joint_metadata must contain exactly six arm joints")
        if not all(isinstance(item, JointMetadata) for item in metadata):
            raise TypeError("joint_metadata must contain JointMetadata entries")
        names = tuple(item.name for item in metadata)
        if names != cls.DEFAULT_JOINT_NAMES:
            raise ValueError("joint_metadata order must match the canonical arm order")
        return metadata

    def _validate_joint_limits(self, joints: JointPositions) -> None:
        violations = [
            metadata.name
            for metadata, position_rad in zip(self._joint_metadata, joints.values)
            if not metadata.lower_limit_rad <= position_rad <= metadata.upper_limit_rad
        ]
        if violations:
            raise RobotArmLimitError(
                "joint positions violate configured limits: {}".format(
                    ", ".join(violations)
                )
            )

    def _read_power_state_locked(
        self, expected_powered: Optional[bool] = None
    ) -> RobotArmState:
        vendor = self._require_vendor()
        is_powered = self._read_vendor_boolean(
            "is_powered_on", vendor.is_powered_on
        )
        if expected_powered is not None and is_powered is not expected_powered:
            message = "MyArm M750 power state did not become {}".format(
                "on" if expected_powered else "off"
            )
            self._record_error(message)
            raise RobotArmProtocolError(message)
        return self._replace_state(
            is_powered=is_powered,
            is_moving=False if not is_powered else self._state.is_moving,
        )

    def _require_vendor(self) -> Any:
        if self._vendor is None:
            raise RobotArmLifecycleError("MyArmM750RobotArm is disconnected")
        return self._vendor

    def _require_powered_for_motion(self) -> None:
        if self._state.is_powered is not True:
            raise RobotArmLifecycleError(
                "MyArmM750RobotArm is not powered on; call power_on() first"
            )

    def _require_initial_measurement(self) -> None:
        if self._state.measured_joint_positions is None:
            raise RobotArmLifecycleError(
                "MyArmM750RobotArm has no measured joint state; call read_state() first"
            )

    def _call_vendor(self, operation_name: str, operation: Callable[[], Any]) -> Any:
        try:
            response = operation()
        except Exception as error:  # Vendor exceptions never cross the adapter boundary.
            message = f"MyArmMControl {operation_name} failed: {error}"
            self._record_error(message)
            raise RobotArmProtocolError(message) from error
        if self._is_vendor_error_response(response):
            message = (
                f"MyArmMControl {operation_name} returned invalid response "
                f"{response!r}"
            )
            self._record_error(message)
            raise RobotArmProtocolError(message)
        return response

    @staticmethod
    def _is_vendor_error_response(response: Any) -> bool:
        return response is None or (
            isinstance(response, (int, float))
            and not isinstance(response, bool)
            and response == -1
        )

    def _read_vendor_boolean(
        self, operation_name: str, operation: Callable[[], Any]
    ) -> bool:
        response = self._call_vendor(operation_name, operation)
        if isinstance(response, bool):
            return response
        if not isinstance(response, int) or response not in (0, 1):
            message = (
                f"MyArmMControl {operation_name} must return 0 or 1, got "
                f"{response!r}"
            )
            self._record_error(message)
            raise RobotArmProtocolError(message)
        return bool(response)

    @classmethod
    def _validate_hardware_angles(cls, response: Any) -> Tuple[float, ...]:
        return cls._six_finite_values(response, "MyArmMControl get_angles() result")

    @classmethod
    def _six_finite_values(
        cls, values: Sequence[float], field_name: str
    ) -> Tuple[float, ...]:
        if not isinstance(values, (list, tuple)) or len(values) != len(
            cls.DEFAULT_JOINT_NAMES
        ):
            raise RobotArmProtocolError(
                f"{field_name} must contain exactly six values: {values!r}"
            )
        try:
            normalized = tuple(float(value) for value in values)
        except (TypeError, ValueError) as error:
            raise RobotArmProtocolError(
                f"{field_name} contains non-numeric values: {values!r}"
            ) from error
        if not all(math.isfinite(value) for value in normalized):
            raise RobotArmProtocolError(
                f"{field_name} contains non-finite values: {values!r}"
            )
        return normalized

    @staticmethod
    def _vendor_speed_from_scale(speed_scale: float) -> int:
        if isinstance(speed_scale, bool):
            raise TypeError("speed_scale must be numeric, not boolean")
        normalized = float(speed_scale)
        if not math.isfinite(normalized) or not 0.0 < normalized <= 1.0:
            raise ValueError("speed_scale must be finite and in the range (0, 1]")
        return max(1, min(100, round(normalized * 100.0)))

    @staticmethod
    def _quantize_hardware_angle_deg(angle_deg: float) -> float:
        """Mirror pymycobot's ``int(angle * 100)`` command resolution."""
        return math.trunc(angle_deg * 100.0) / 100.0

    def _replace_state(self, **changes) -> RobotArmState:
        self._state = replace(
            self._state,
            sequence=self._state.sequence + 1,
            consecutive_error_count=0,
            last_error_message=None,
            **changes,
        )
        return self._state

    def _record_error(self, message: str) -> None:
        self._state = replace(
            self._state,
            sequence=self._state.sequence + 1,
            consecutive_error_count=self._state.consecutive_error_count + 1,
            last_error_message=message,
        )

    def _disconnect_vendor_after_failed_connect(self, vendor: Any) -> None:
        try:
            self._close_vendor_transport(vendor)
        except Exception as error:  # noqa: BLE001 - vendor close errors are opaque.
            self._record_error(f"failed to close serial after connect failure: {error}")
        self._vendor = None
        self._state = replace(
            self._state,
            sequence=self._state.sequence + 1,
            is_connected=False,
            is_powered=None,
            is_moving=None,
        )

    @staticmethod
    def _close_vendor_transport(vendor: Any) -> None:
        close_method = getattr(vendor, "close", None)
        if callable(close_method):
            close_method()
            return
        serial_transport = getattr(vendor, "_serial_port", None)
        serial_close = getattr(serial_transport, "close", None)
        if callable(serial_close):
            serial_close()
            return
        raise RobotArmConnectionError(
            "MyArmMControl exposes no closable serial transport"
        )


# The package path already communicates the adapter role.  Keep this alias so
# existing imports remain usable until the next intentional breaking release.
MyArmM750RobotArmAdapter = MyArmM750RobotArm
