"""Deterministic in-memory implementation of :class:`RobotArmInterface`."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import replace
from typing import Optional, Sequence, Tuple

from myarm_sdk.core import (
    GripperCommand,
    GripperState,
    JointMetadata,
    JointPositions,
    RobotArmCommand,
    RobotArmLifecycleError,
    RobotArmLimitError,
    RobotArmState,
)


class FakeRobotArm:
    """A stateful memory robot that applies accepted targets immediately.

    It deliberately does not interpolate or invent a trajectory.  A planner,
    preview player or executor decides when each target is sent; this fake then
    stores exactly that accepted target as both command and measured state.
    """

    DEFAULT_JOINT_NAMES = (
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_flex_joint",
        "forearm_roll_joint",
        "wrist_flex_joint",
        "wrist_roll_joint",
    )
    DEFAULT_HOME_JOINT_POSITIONS = JointPositions(
        (0.0, -0.35, 0.70, 0.0, -0.35, 0.0)
    )

    def __init__(
        self,
        initial_joint_positions: Optional[JointPositions] = None,
        joint_metadata: Sequence[JointMetadata] = (),
        start_connected: bool = True,
        start_powered: bool = True,
        initial_gripper_opening_width_m: float = 0.0,
    ) -> None:
        if initial_joint_positions is not None and not isinstance(
            initial_joint_positions, JointPositions
        ):
            raise TypeError("initial_joint_positions must be JointPositions or None")
        if not isinstance(start_connected, bool):
            raise TypeError("start_connected must be boolean")
        if not isinstance(start_powered, bool):
            raise TypeError("start_powered must be boolean")
        initial_gripper_opening_width_m = self._validate_gripper_opening(
            initial_gripper_opening_width_m
        )
        self._joint_metadata = self._validate_joint_metadata(joint_metadata)
        initial = (
            initial_joint_positions
            if initial_joint_positions is not None
            else self.DEFAULT_HOME_JOINT_POSITIONS
        )
        self._validate_joint_limits(initial)
        now_s = time.monotonic()
        self._state = RobotArmState(
            source="fake_robot_arm",
            is_connected=start_connected,
            is_powered=start_powered,
            is_moving=False,
            measured_joint_positions=initial,
            measured_at_monotonic_s=now_s,
            gripper_state=GripperState(
                opening_width_m=initial_gripper_opening_width_m,
                is_enabled=start_powered,
                is_moving=False,
                measured_at_monotonic_s=now_s,
            ),
        )
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
            return self._state.is_connected

    def connect(self) -> RobotArmState:
        with self._lock:
            if self._state.is_connected:
                return self._state
            return self._replace_state(is_connected=True, is_moving=False)

    def disconnect(self) -> RobotArmState:
        with self._lock:
            if not self._state.is_connected:
                return self._state
            return self._replace_state(is_connected=False, is_moving=False)

    def read_state(self) -> RobotArmState:
        with self._lock:
            self._require_connected()
            return self._replace_state(
                measured_at_monotonic_s=time.monotonic(),
                is_moving=False,
            )

    def write_joint_positions(
        self, target: JointPositions, speed_scale: float = 0.5
    ) -> RobotArmCommand:
        with self._lock:
            self._require_ready_for_motion()
            self._validate_speed_scale(speed_scale)
            self._validate_joint_limits(target)
            now_s = time.monotonic()
            next_sequence = self._state.sequence + 1
            command = RobotArmCommand(
                requested_joint_positions=target,
                accepted_joint_positions=target,
                speed_scale=speed_scale,
                issued_at_monotonic_s=now_s,
                sequence=next_sequence,
            )
            self._state = replace(
                self._state,
                measured_joint_positions=target,
                measured_at_monotonic_s=now_s,
                last_command=command,
                is_moving=False,
                sequence=next_sequence,
                consecutive_error_count=0,
                last_error_message=None,
            )
            return command

    def stop(self) -> RobotArmState:
        with self._lock:
            self._require_connected()
            return self._replace_state(
                is_moving=False,
                gripper_state=self._replace_gripper_state(is_moving=False),
            )

    def power_on(self) -> RobotArmState:
        with self._lock:
            self._require_connected()
            return self._replace_state(
                is_powered=True,
                is_moving=False,
                gripper_state=self._replace_gripper_state(is_enabled=True),
            )

    def power_off(self) -> RobotArmState:
        with self._lock:
            self._require_connected()
            return self._replace_state(
                is_powered=False,
                is_moving=False,
                gripper_state=self._replace_gripper_state(
                    is_enabled=False, is_moving=False
                ),
            )

    def read_power_state(self) -> RobotArmState:
        with self._lock:
            self._require_connected()
            return self._replace_state()

    def read_motion_state(self) -> RobotArmState:
        with self._lock:
            self._require_connected()
            return self._replace_state(is_moving=False)

    def read_gripper_state(self) -> GripperState:
        with self._lock:
            self._require_connected()
            state = self._replace_gripper_state(is_moving=False)
            self._replace_state(gripper_state=state)
            return state

    def enable_gripper(self) -> RobotArmState:
        with self._lock:
            self._require_ready_for_motion()
            return self._replace_state(
                gripper_state=self._replace_gripper_state(is_enabled=True)
            )

    def write_gripper_opening(
        self, opening_width_m: float, speed_scale: float = 0.5
    ) -> GripperCommand:
        with self._lock:
            self._require_ready_for_motion()
            self._validate_speed_scale(speed_scale)
            opening_width_m = self._validate_gripper_opening(opening_width_m)
            gripper = self._state.gripper_state
            if gripper is None or gripper.is_enabled is not True:
                raise RobotArmLifecycleError("FakeRobotArm gripper is not enabled")
            now_s = time.monotonic()
            command = GripperCommand(
                requested_opening_width_m=opening_width_m,
                accepted_opening_width_m=opening_width_m,
                speed_scale=speed_scale,
                issued_at_monotonic_s=now_s,
                sequence=gripper.sequence + 1,
            )
            return_command_state = GripperState(
                opening_width_m=opening_width_m,
                is_enabled=True,
                is_moving=False,
                measured_at_monotonic_s=now_s,
                last_command=command,
                sequence=command.sequence,
            )
            self._replace_state(gripper_state=return_command_state)
            return command

    def read_gripper_motion_state(self) -> GripperState:
        return self.read_gripper_state()

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

    @classmethod
    def _validate_joint_metadata(
        cls, joint_metadata: Sequence[JointMetadata]
    ) -> Tuple[JointMetadata, ...]:
        metadata = tuple(joint_metadata)
        if not metadata:
            return metadata
        if not all(isinstance(item, JointMetadata) for item in metadata):
            raise TypeError("joint_metadata must contain JointMetadata entries")
        if len(metadata) != len(cls.DEFAULT_JOINT_NAMES):
            raise ValueError("joint_metadata must contain exactly six arm joints")
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

    @staticmethod
    def _validate_speed_scale(speed_scale: float) -> None:
        if isinstance(speed_scale, bool):
            raise TypeError("speed_scale must be numeric, not boolean")
        normalized = float(speed_scale)
        if not math.isfinite(normalized) or not 0.0 < normalized <= 1.0:
            raise ValueError("speed_scale must be finite and in the range (0, 1]")

    @staticmethod
    def _validate_gripper_opening(opening_width_m: float) -> float:
        if isinstance(opening_width_m, bool):
            raise TypeError("opening_width_m must be numeric, not boolean")
        normalized = float(opening_width_m)
        if not math.isfinite(normalized) or not 0.0 <= normalized <= 0.08:
            raise ValueError("opening_width_m must be in [0, 0.08] metres")
        return normalized

    def _require_connected(self) -> None:
        if not self._state.is_connected:
            raise RobotArmLifecycleError("FakeRobotArm is disconnected")

    def _require_ready_for_motion(self) -> None:
        self._require_connected()
        if self._state.is_powered is not True:
            raise RobotArmLifecycleError("FakeRobotArm is not powered on")

    def _replace_state(self, **changes) -> RobotArmState:
        self._state = replace(
            self._state,
            sequence=self._state.sequence + 1,
            consecutive_error_count=0,
            last_error_message=None,
            **changes,
        )
        return self._state

    def _replace_gripper_state(self, **changes) -> GripperState:
        current = self._state.gripper_state
        if current is None:
            current = GripperState()
        return replace(
            current,
            measured_at_monotonic_s=(
                time.monotonic()
                if current.opening_width_m is not None
                else current.measured_at_monotonic_s
            ),
            sequence=current.sequence + 1,
            consecutive_error_count=0,
            last_error_message=None,
            **changes,
        )


# The package path already communicates the adapter role.  Keep this alias so
# existing imports remain usable until the next intentional breaking release.
FakeRobotArmAdapter = FakeRobotArm
