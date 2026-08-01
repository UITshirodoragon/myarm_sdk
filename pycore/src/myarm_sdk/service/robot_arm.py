"""Stateful robot-arm capability service used by a ROS driver boundary."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Tuple

from myarm_sdk.core import (
    GripperCommand,
    GripperState,
    JointMetadata,
    JointPositions,
    RobotArmCommand,
    RobotArmError,
    RobotArmState,
    load_sdk_yaml,
    load_urdf_joint_metadata,
)
from myarm_sdk.core.validation import require_enabled
from myarm_sdk.plugin_adapter.robot_arm import FakeRobotArm, MyArmM750RobotArm
from myarm_sdk.port_interface import RobotArmInterface


class RobotArmServiceError(RuntimeError):
    """A robot-arm service policy or configuration rejected an operation."""


@dataclass(frozen=True)
class RobotArmFeedback:
    """One non-throwing hardware feedback read for a driver boundary.

    This is intentionally feedback-only.  Trajectory scheduling, command
    queuing and arbitration belong to motion execution, not the robot service.
    """

    state: RobotArmState
    measured_joint_positions: Optional[JointPositions]
    measurement_age_s: Optional[float]
    measured_state_fresh: bool
    feedback_updated: bool
    feedback_error: Optional[str]


@dataclass(frozen=True)
class RobotArmGripperFeedback:
    """One non-throwing gripper feedback read from the RobotArm capability."""

    state: GripperState
    feedback_updated: bool
    feedback_error: Optional[str]


class RobotArmService:
    """Own one robot-arm port as a feedback/lifecycle and setpoint gateway.

    It deliberately has no trajectory, goal, preemption or command-arbitration
    state.  A single driver node owns the transport mailbox, while
    ``MotionExecutionService`` owns high-level motion state and timing.
    """

    def __init__(
        self,
        robot_arm: RobotArmInterface,
        update_rate_hz: float,
        feedback_stale_after_s: float,
        default_speed_scale: float = 0.5,
        accepts_execution_setpoints: bool = False,
        accepts_gripper_commands: bool = False,
        power_on_on_connect: bool = False,
        joint_metadata: Tuple[JointMetadata, ...] = (),
    ) -> None:
        self._robot_arm = robot_arm
        self._joint_names = self._validate_joint_names(robot_arm.joint_names)
        self._update_rate_hz = self._positive_finite(
            update_rate_hz, "update_rate_hz"
        )
        self._feedback_stale_after_s = self._positive_finite(
            feedback_stale_after_s, "feedback_stale_after_s"
        )
        self._default_speed_scale = self._speed_scale(default_speed_scale)
        if not isinstance(accepts_execution_setpoints, bool):
            raise TypeError("accepts_execution_setpoints must be boolean")
        if not isinstance(power_on_on_connect, bool):
            raise TypeError("power_on_on_connect must be boolean")
        if not isinstance(accepts_gripper_commands, bool):
            raise TypeError("accepts_gripper_commands must be boolean")
        self._accepts_execution_setpoints = accepts_execution_setpoints
        self._accepts_gripper_commands = accepts_gripper_commands
        self._power_on_on_connect = power_on_on_connect
        self._joint_metadata = self._validate_joint_metadata(
            joint_metadata, self._joint_names
        )
        self._operation_lock = threading.RLock()

    @classmethod
    def from_config(
        cls,
        service_config: Mapping[str, Any],
        package_share_directory: Callable[[str], str],
        robot_config: Mapping[str, Any],
        vendor_factory: Optional[Callable[..., Any]] = None,
    ) -> RobotArmService:
        """Create the selected fake or physical robot backend without I/O.

        ``robot_config`` is the shared top-level ``robot`` manifest section.
        Its URDF supplies canonical joint metadata and its named pose supplies
        the fake backend's initial state.  ``package_share_directory`` is
        injected by the ROS composition boundary, so this SDK remains free of
        ROS imports.
        """
        require_enabled(service_config, "robot_arm")
        if not callable(package_share_directory):
            raise TypeError("package_share_directory must be callable")

        plugin_adapter = cls._required_string(
            service_config.get("plugin_adapter"), "robot_arm plugin_adapter"
        )
        if plugin_adapter not in ("fake_robot_arm", "myarm_m750_robot_arm"):
            raise ValueError(
                "robot_arm plugin_adapter must be fake_robot_arm or "
                "myarm_m750_robot_arm"
            )
        adapter_config = load_sdk_yaml(
            cls._required_string(service_config.get("plugin_config"), "plugin_config")
        )
        if adapter_config.get("plugin_adapter") != plugin_adapter:
            raise ValueError(
                "robot_arm plugin_config plugin_adapter must match service plugin_adapter"
            )

        joint_names = cls._joint_names_from_robot_config(robot_config)
        urdf_path = cls._resolve_urdf_path(robot_config, package_share_directory)
        joint_metadata = load_urdf_joint_metadata(urdf_path, joint_names)
        initial_joint_positions = cls._initial_named_pose(robot_config, service_config)
        cls._validate_pose_limits(initial_joint_positions, joint_metadata)

        transport_config = cls._mapping(
            service_config.get("transport"), "robot_arm transport"
        )
        default_speed_scale = cls._speed_scale(
            transport_config.get("default_speed_scale")
        )
        accept_internal_setpoints = cls._boolean(
            transport_config.get("accept_internal_setpoints"),
            "robot_arm transport.accept_internal_setpoints",
        )
        allow_physical_motion = cls._boolean(
            transport_config.get("allow_physical_motion", False),
            "robot_arm transport.allow_physical_motion",
        )
        gripper_config = cls._mapping(
            service_config.get("gripper"), "robot_arm gripper"
        )
        gripper_enabled = cls._boolean(
            gripper_config.get("enabled"), "robot_arm gripper.enabled"
        )
        allow_physical_gripper_actuation = cls._boolean(
            gripper_config.get("allow_physical_actuation", False),
            "robot_arm gripper.allow_physical_actuation",
        )
        gripper_initial_opening_width_m = cls._opening_width(
            gripper_config.get("initial_opening_width_m", 0.0),
            "robot_arm gripper.initial_opening_width_m",
        )
        feedback_config = cls._mapping(
            service_config.get("feedback"), "robot_arm feedback"
        )
        feedback_stale_after_s = cls._positive_finite(
            feedback_config.get("stale_after_s"), "robot_arm feedback.stale_after_s"
        )
        update_rate_hz = cls._positive_finite(
            service_config.get("update_rate_hz"), "robot_arm update_rate_hz"
        )

        power_on_on_connect = False
        if plugin_adapter == "fake_robot_arm":
            robot_arm = FakeRobotArm(
                initial_joint_positions=initial_joint_positions,
                joint_metadata=joint_metadata,
                start_connected=cls._boolean(
                    adapter_config.get("start_connected", True),
                    "fake_robot_arm start_connected",
                ),
                start_powered=cls._boolean(
                    adapter_config.get("start_powered", True),
                    "fake_robot_arm start_powered",
                ),
                initial_gripper_opening_width_m=gripper_initial_opening_width_m,
            )
            accepts_execution_setpoints = accept_internal_setpoints
            accepts_gripper_commands = gripper_enabled
        else:
            robot_arm, power_on_on_connect = cls._physical_robot_from_config(
                adapter_config=adapter_config,
                joint_metadata=joint_metadata,
                gripper_config=gripper_config,
                vendor_factory=vendor_factory,
            )
            accepts_execution_setpoints = (
                accept_internal_setpoints and allow_physical_motion
            )
            accepts_gripper_commands = (
                gripper_enabled
                and allow_physical_motion
                and allow_physical_gripper_actuation
            )

        return cls(
            robot_arm=robot_arm,
            update_rate_hz=update_rate_hz,
            feedback_stale_after_s=feedback_stale_after_s,
            default_speed_scale=default_speed_scale,
            accepts_execution_setpoints=accepts_execution_setpoints,
            accepts_gripper_commands=accepts_gripper_commands,
            power_on_on_connect=power_on_on_connect,
            joint_metadata=joint_metadata,
        )

    @property
    def joint_names(self) -> Tuple[str, ...]:
        """Return the canonical URDF order expected by this driver."""
        return self._joint_names

    @property
    def joint_metadata(self) -> Tuple[JointMetadata, ...]:
        """Return the exact ordered hard-limit metadata loaded from URDF."""
        return self._joint_metadata

    @property
    def state(self) -> RobotArmState:
        """Return cached adapter state without reading hardware."""
        with self._operation_lock:
            return self._robot_arm.state

    @property
    def update_rate_hz(self) -> float:
        return self._update_rate_hz

    @property
    def feedback_stale_after_s(self) -> float:
        return self._feedback_stale_after_s

    @property
    def default_speed_scale(self) -> float:
        return self._default_speed_scale

    @property
    def accepts_execution_setpoints(self) -> bool:
        """Whether the driver may accept its internal executor setpoint stream."""
        return self._accepts_execution_setpoints

    @property
    def accepts_gripper_commands(self) -> bool:
        """Whether physical or fake gripper commands are currently authorized."""
        return self._accepts_gripper_commands

    def connect(self) -> RobotArmState:
        """Connect explicitly; optional power-on occurs only after success."""
        with self._operation_lock:
            was_connected = self._robot_arm.is_connected
            state = self._robot_arm.connect()
            if self._power_on_on_connect and not was_connected:
                state = self._robot_arm.power_on()
            return state

    def disconnect(self) -> RobotArmState:
        """Release the backend transport without changing power implicitly."""
        with self._operation_lock:
            return self._robot_arm.disconnect()

    def refresh_state(self) -> RobotArmState:
        """Explicitly read current feedback and let lifecycle errors propagate."""
        with self._operation_lock:
            return self._robot_arm.read_state()

    def read_feedback(
        self, now_monotonic_s: Optional[float] = None
    ) -> RobotArmFeedback:
        """Read feedback without letting a periodic driver timer terminate.

        The returned state is always the latest immutable cache, including when
        the adapter reports an error.  No command is scheduled or emitted here.
        """
        with self._operation_lock:
            now_s = self._now_monotonic_s(now_monotonic_s)
            feedback_updated = False
            feedback_error: Optional[str] = None
            try:
                state = self._robot_arm.read_state()
                feedback_updated = True
            except RobotArmError as error:
                state = self._robot_arm.state
                feedback_error = str(error)
            return self._feedback_from_state(
                state=state,
                now_monotonic_s=now_s,
                feedback_updated=feedback_updated,
                feedback_error=feedback_error,
            )

    def send_joint_setpoint(
        self, target: JointPositions, speed_scale: Optional[float] = None
    ) -> RobotArmCommand:
        """Send one already-authorized atomic joint setpoint to the robot port.

        This gateway neither interpolates nor queues a trajectory.  Only the
        driver node calls it for an internal stream emitted by motion execution.
        """
        with self._operation_lock:
            if not self._accepts_execution_setpoints:
                raise RobotArmServiceError(
                    "internal execution setpoints are disabled by robot_arm transport policy"
                )
            if not isinstance(target, JointPositions):
                raise TypeError("target must be JointPositions")
            normalized_speed_scale = (
                self._default_speed_scale
                if speed_scale is None
                else self._speed_scale(speed_scale)
            )
            return self._robot_arm.write_joint_positions(
                target,
                speed_scale=normalized_speed_scale,
            )

    def read_gripper_feedback(self) -> RobotArmGripperFeedback:
        """Read gripper feedback without allowing a driver timer to terminate."""
        with self._operation_lock:
            try:
                state = self._robot_arm.read_gripper_state()
                return RobotArmGripperFeedback(
                    state=state, feedback_updated=True, feedback_error=None
                )
            except RobotArmError as error:
                cached = self._robot_arm.state.gripper_state or GripperState()
                return RobotArmGripperFeedback(
                    state=cached, feedback_updated=False, feedback_error=str(error)
                )

    def enable_gripper(self) -> RobotArmState:
        with self._operation_lock:
            if not self._accepts_gripper_commands:
                raise RobotArmServiceError(
                    "gripper commands are disabled by robot_arm transport policy"
                )
            return self._robot_arm.enable_gripper()

    def send_gripper_opening(
        self, opening_width_m: float, speed_scale: Optional[float] = None
    ) -> GripperCommand:
        with self._operation_lock:
            if not self._accepts_gripper_commands:
                raise RobotArmServiceError(
                    "gripper commands are disabled by robot_arm transport policy"
                )
            normalized_speed_scale = (
                self._default_speed_scale
                if speed_scale is None
                else self._speed_scale(speed_scale)
            )
            return self._robot_arm.write_gripper_opening(
                opening_width_m, speed_scale=normalized_speed_scale
            )

    def stop(self) -> RobotArmState:
        """Request the backend software motion stop."""
        with self._operation_lock:
            return self._robot_arm.stop()

    def power_on(self) -> RobotArmState:
        with self._operation_lock:
            return self._robot_arm.power_on()

    def power_off(self) -> RobotArmState:
        """Request power-off at the backend."""
        with self._operation_lock:
            return self._robot_arm.power_off()

    def _feedback_from_state(
        self,
        state: RobotArmState,
        now_monotonic_s: float,
        feedback_updated: bool,
        feedback_error: Optional[str],
    ) -> RobotArmFeedback:
        measurement_age_s = self._measurement_age_s(state, now_monotonic_s)
        measured_state_fresh = (
            measurement_age_s is not None
            and measurement_age_s <= self._feedback_stale_after_s
        )
        return RobotArmFeedback(
            state=state,
            measured_joint_positions=state.measured_joint_positions,
            measurement_age_s=measurement_age_s,
            measured_state_fresh=measured_state_fresh,
            feedback_updated=feedback_updated,
            feedback_error=feedback_error,
        )

    @classmethod
    def _physical_robot_from_config(
        cls,
        adapter_config: Mapping[str, Any],
        joint_metadata: Tuple[JointMetadata, ...],
        gripper_config: Mapping[str, Any],
        vendor_factory: Optional[Callable[..., Any]],
    ) -> Tuple[MyArmM750RobotArm, bool]:
        connection = cls._mapping(
            adapter_config.get("connection"), "myarm_m750_robot_arm connection"
        )
        adapter_command = cls._mapping(
            adapter_config.get("command"), "myarm_m750_robot_arm command"
        )
        if adapter_command.get("fresh_mode", "latest") != "latest":
            raise ValueError("myarm_m750_robot_arm command.fresh_mode must be latest")
        joint_convention = cls._mapping(
            adapter_config.get("joint_convention"),
            "myarm_m750_robot_arm joint_convention",
        )
        offsets = joint_convention.get("model_to_hardware_offsets_rad")
        if offsets is None:
            raise ValueError(
                "myarm_m750_robot_arm joint_convention requires "
                "model_to_hardware_offsets_rad"
            )
        lifecycle = cls._mapping_or_empty(
            adapter_config.get("lifecycle"), "myarm_m750_robot_arm lifecycle"
        )
        power_on_on_connect = cls._boolean(
            lifecycle.get("power_on_on_connect", False),
            "myarm_m750_robot_arm lifecycle.power_on_on_connect",
        )
        return (
            MyArmM750RobotArm(
                serial_port=cls._required_string(
                    connection.get("serial_port"),
                    "myarm_m750_robot_arm connection.serial_port",
                ),
                joint_metadata=joint_metadata,
                baudrate=connection.get("baudrate", MyArmM750RobotArm.DEFAULT_BAUDRATE),
                timeout_s=connection.get(
                    "timeout_s", MyArmM750RobotArm.DEFAULT_TIMEOUT_S
                ),
                debug=cls._boolean(
                    connection.get("debug", False),
                    "myarm_m750_robot_arm connection.debug",
                ),
                model_to_hardware_offsets_rad=offsets,
                gripper_enabled=cls._boolean(
                    gripper_config.get("enabled"), "robot_arm gripper.enabled"
                ),
                gripper_vendor_value_at_closed=cls._gripper_vendor_value(
                    gripper_config.get("vendor_value_at_closed"),
                    "robot_arm gripper.vendor_value_at_closed",
                ),
                gripper_vendor_value_at_open=cls._gripper_vendor_value(
                    gripper_config.get("vendor_value_at_open"),
                    "robot_arm gripper.vendor_value_at_open",
                ),
                vendor_factory=vendor_factory,
            ),
            power_on_on_connect,
        )

    @classmethod
    def _joint_names_from_robot_config(
        cls, robot_config: Mapping[str, Any]
    ) -> Tuple[str, ...]:
        joint_order = cls._mapping(robot_config.get("joint_order"), "robot.joint_order")
        if joint_order.get("source") != "urdf":
            raise ValueError("robot.joint_order.source must be 'urdf'")
        return cls._validate_joint_names(joint_order.get("names"))

    @classmethod
    def _resolve_urdf_path(
        cls,
        robot_config: Mapping[str, Any],
        package_share_directory: Callable[[str], str],
    ) -> Path:
        description = cls._mapping(
            robot_config.get("robot_description"), "robot.robot_description"
        )
        package_name = cls._required_string(
            description.get("package"), "robot.robot_description.package"
        )
        relative_path = Path(
            cls._required_string(
                description.get("relative_path"),
                "robot.robot_description.relative_path",
            )
        )
        if relative_path.is_absolute():
            raise ValueError("robot.robot_description.relative_path must be relative")
        # Keep the lexical package-share path rather than resolving it.  A
        # colcon ``--symlink-install`` workspace resolves a URDF inside the
        # share directory back into the source tree, which would otherwise
        # look like a false directory escape.
        if ".." in relative_path.parts:
            raise ValueError("robot.robot_description.relative_path escapes package share")
        package_share = Path(package_share_directory(package_name))
        return package_share / relative_path

    @classmethod
    def _initial_named_pose(
        cls,
        robot_config: Mapping[str, Any],
        service_config: Mapping[str, Any],
    ) -> JointPositions:
        pose_name = cls._required_string(
            service_config.get("initial_named_pose"), "robot_arm initial_named_pose"
        )
        named_poses = cls._mapping(robot_config.get("named_poses"), "robot.named_poses")
        pose_config = cls._mapping(
            named_poses.get(pose_name), f"robot.named_poses.{pose_name}"
        )
        try:
            return JointPositions(pose_config["positions_rad"])
        except KeyError as error:
            raise ValueError(
                f"robot.named_poses.{pose_name} requires positions_rad"
            ) from error

    @staticmethod
    def _validate_pose_limits(
        positions: JointPositions, metadata: Tuple[JointMetadata, ...]
    ) -> None:
        violations = tuple(
            item.name
            for item, position_rad in zip(metadata, positions.values)
            if position_rad < item.lower_limit_rad or position_rad > item.upper_limit_rad
        )
        if violations:
            raise ValueError(
                "initial_named_pose violates URDF joint limits: {}".format(
                    ", ".join(violations)
                )
            )

    @staticmethod
    def _measurement_age_s(
        state: RobotArmState, now_monotonic_s: float
    ) -> Optional[float]:
        if state.measured_at_monotonic_s is None:
            return None
        return max(0.0, now_monotonic_s - state.measured_at_monotonic_s)

    @staticmethod
    def _now_monotonic_s(now_monotonic_s: Optional[float]) -> float:
        now_s = time.monotonic() if now_monotonic_s is None else now_monotonic_s
        if isinstance(now_s, bool):
            raise TypeError("now_monotonic_s must be numeric, not boolean")
        try:
            normalized = float(now_s)
        except (TypeError, ValueError) as error:
            raise TypeError("now_monotonic_s must be numeric") from error
        if not math.isfinite(normalized):
            raise ValueError("now_monotonic_s must be finite")
        return normalized

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    @staticmethod
    def _mapping_or_empty(value: Any, name: str) -> Mapping[str, Any]:
        if value is None:
            return {}
        return RobotArmService._mapping(value, name)

    @staticmethod
    def _required_string(value: Any, name: str) -> str:
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"{name} must be a non-empty string")
        return value

    @staticmethod
    def _boolean(value: Any, name: str) -> bool:
        if not isinstance(value, bool):
            raise TypeError(f"{name} must be boolean")
        return value

    @staticmethod
    def _positive_finite(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric, not boolean")
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or normalized <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return normalized

    @staticmethod
    def _speed_scale(value: Any) -> float:
        if isinstance(value, bool):
            raise TypeError("speed_scale must be numeric, not boolean")
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError("speed_scale must be numeric") from error
        if not math.isfinite(normalized) or not 0.0 < normalized <= 1.0:
            raise ValueError("speed_scale must be finite and in the range (0, 1]")
        return normalized

    @staticmethod
    def _opening_width(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric, not boolean")
        try:
            normalized = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(normalized) or not 0.0 <= normalized <= 0.08:
            raise ValueError(f"{name} must be in [0, 0.08] metres")
        return normalized

    @staticmethod
    def _gripper_vendor_value(value: Any, name: str) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise TypeError(f"{name} must be an integer")
        if not 0 <= value <= 100:
            raise ValueError(f"{name} must be in the range 0..100")
        return value

    @staticmethod
    def _validate_joint_names(joint_names: Any) -> Tuple[str, ...]:
        if isinstance(joint_names, str) or not isinstance(joint_names, (list, tuple)):
            raise TypeError("robot arm joint_names must be a sequence")
        names = tuple(joint_names)
        if len(names) != 6:
            raise ValueError("MyArm M750 requires exactly six canonical joint names")
        if not all(isinstance(name, str) and name.strip() for name in names):
            raise ValueError("robot arm joint_names must contain non-empty strings")
        if len(set(names)) != len(names):
            raise ValueError("robot arm joint_names must be unique")
        return names

    @staticmethod
    def _validate_joint_metadata(
        metadata: Tuple[JointMetadata, ...], joint_names: Tuple[str, ...]
    ) -> Tuple[JointMetadata, ...]:
        normalized = tuple(metadata)
        if not normalized:
            return normalized
        if not all(isinstance(item, JointMetadata) for item in normalized):
            raise TypeError("joint_metadata must contain JointMetadata entries")
        if tuple(item.name for item in normalized) != joint_names:
            raise ValueError("joint_metadata order must match robot arm joint_names")
        return normalized
