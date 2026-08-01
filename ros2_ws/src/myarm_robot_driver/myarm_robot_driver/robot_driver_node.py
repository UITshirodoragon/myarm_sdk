"""ROS 2 hardware boundary for MyArm M750 feedback, lifecycle and setpoints.

The node is the sole serial owner.  It deliberately knows nothing about
Cartesian goals, trajectories, timing profiles or preemption policy; those
belong to ``myarm_motion_execution``.  Its optional input is a private,
already-time-parameterized setpoint stream from that node.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Mapping, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from myarm_sdk.core import JointPositions, RobotArmCommand, RobotArmState, load_sdk_yaml
from myarm_sdk.service import RobotArmService
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .joint_state_mapping import canonical_joint_positions_from_names


class MyArmRobotDriverNode(Node):
    """Publish canonical feedback and accept only internal executor setpoints."""

    _SERVICES_CONFIG = "service/config/services.yaml"

    def __init__(self, robot_arm_service: Optional[Any] = None) -> None:
        super().__init__("myarm_robot_driver")
        services_config = load_sdk_yaml(self._SERVICES_CONFIG)
        self._robot_config = self._mapping(services_config.get("robot"), "robot")
        self._service_config = self._robot_arm_config(services_config)
        self._topics = self._mapping(self._service_config.get("topics"), "robot topics")

        self._service = robot_arm_service or RobotArmService.from_config(
            robot_config=self._robot_config,
            service_config=self._service_config,
            package_share_directory=get_package_share_directory,
        )
        self._joint_names = tuple(self._service.joint_names)
        self._validate_joint_names(self._joint_names)
        self._accepts_execution_setpoints = self._required_bool(
            self._service.accepts_execution_setpoints,
            "RobotArmService.accepts_execution_setpoints",
        )
        self._pending_setpoint: Optional[JointPositions] = None
        self._pending_setpoint_lock = threading.RLock()

        measured_topic = self._required_topic("measured_joint_state")
        self._measured_joint_state_publisher = self.create_publisher(
            JointState, measured_topic, 10
        )
        visualization_topic = self._optional_topic("visualization_joint_state")
        self._visualization_joint_state_publisher = None
        if visualization_topic and visualization_topic != measured_topic:
            self._visualization_joint_state_publisher = self.create_publisher(
                JointState, visualization_topic, 10
            )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, self._required_topic("diagnostics"), 10
        )

        self._setpoint_subscription = None
        if self._accepts_execution_setpoints:
            self._setpoint_subscription = self.create_subscription(
                JointState,
                self._required_topic("internal_setpoint"),
                self._internal_setpoint_callback,
                10,
            )
        else:
            self.get_logger().info(
                "Internal execution setpoints are disabled by robot transport policy."
            )

        self._stop_service = self.create_service(
            Trigger, self._required_topic("stop_service"), self._stop_callback
        )
        self._power_on_service = self.create_service(
            Trigger, self._required_topic("power_on_service"), self._power_on_callback
        )
        self._power_off_service = self.create_service(
            Trigger, self._required_topic("power_off_service"), self._power_off_callback
        )

        update_rate_hz = self._positive_float(
            self._service.update_rate_hz, "robot_arm update_rate_hz"
        )
        self.create_timer(1.0 / update_rate_hz, self._poll_and_publish)
        self._connect_at_startup()
        self.get_logger().info(
            f"myarm_robot_driver is running at {self._format_number(update_rate_hz)} Hz; actual state is published on {measured_topic}."
        )

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    @classmethod
    def _robot_arm_config(cls, services_config: Mapping[str, Any]) -> Mapping[str, Any]:
        services = cls._mapping(services_config.get("services"), "services")
        robot_arm = cls._mapping(services.get("robot_arm"), "services.robot_arm")
        if not cls._required_bool(robot_arm.get("enabled"), "robot_arm.enabled"):
            raise RuntimeError("Robot-arm service is disabled in services.yaml")
        return robot_arm

    @staticmethod
    def _required_bool(value: Any, name: str) -> bool:
        if not isinstance(value, bool):
            raise TypeError(f"{name} must be boolean")
        return value

    @staticmethod
    def _positive_float(value: Any, name: str) -> float:
        if isinstance(value, bool):
            raise TypeError(f"{name} must be numeric, not boolean")
        try:
            number = float(value)
        except (TypeError, ValueError) as error:
            raise TypeError(f"{name} must be numeric") from error
        if not math.isfinite(number) or number <= 0.0:
            raise ValueError(f"{name} must be finite and positive")
        return number

    def _required_topic(self, name: str) -> str:
        topic = self._optional_topic(name)
        if topic is None:
            raise ValueError(f"robot topics.{name} must be a non-empty string")
        return topic

    def _optional_topic(self, name: str) -> Optional[str]:
        value = self._topics.get(name)
        if value is None:
            return None
        if not isinstance(value, str):
            raise TypeError(f"robot topics.{name} must be a string or null")
        topic = value.strip()
        return topic or None

    @staticmethod
    def _validate_joint_names(joint_names) -> None:
        if len(joint_names) != 6:
            raise ValueError("RobotArmService must expose exactly six arm joints")
        if not all(isinstance(name, str) and name for name in joint_names):
            raise ValueError("RobotArmService joint names must be non-empty strings")
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("RobotArmService joint names must be unique")

    def _connect_at_startup(self) -> None:
        """Connect once without implicitly commanding motion."""
        try:
            state = self._service.connect()
        except Exception as error:  # noqa: BLE001 - preserve hardware visibility.
            detail = f"startup connection failed: {error}"
            self.get_logger().error(detail)
            self._publish_diagnostics(
                state=self._safe_state(),
                feedback_age_s=None,
                feedback_fresh=False,
                feedback_updated=False,
                command=None,
                command_submitted=False,
                boundary_error=detail,
            )
            return
        self._publish_state(state)

    def _internal_setpoint_callback(self, message: JointState) -> None:
        """Store only the newest transport setpoint; no serial I/O in callback."""
        try:
            target = canonical_joint_positions_from_names(
                names=message.name,
                positions=message.position,
                canonical_joint_names=self._joint_names,
            )
        except Exception as error:  # noqa: BLE001 - publish clear boundary diagnostics.
            detail = f"internal setpoint rejected: {error}"
            self.get_logger().warning(detail)
            self._publish_boundary_error("invalid_internal_setpoint", detail)
            return
        with self._pending_setpoint_lock:
            self._pending_setpoint = target

    def _take_pending_setpoint(self) -> Optional[JointPositions]:
        with self._pending_setpoint_lock:
            target = self._pending_setpoint
            self._pending_setpoint = None
            return target

    def _clear_pending_setpoint(self) -> None:
        with self._pending_setpoint_lock:
            self._pending_setpoint = None

    def _poll_and_publish(self) -> None:
        feedback = self._service.read_feedback(now_monotonic_s=time.monotonic())
        self._publish_state(feedback.state)

        command = None
        command_submitted = False
        command_error = None
        if feedback.feedback_error is None and feedback.measured_state_fresh:
            target = self._take_pending_setpoint()
            if target is not None:
                command_submitted = True
                try:
                    command = self._service.send_joint_setpoint(target)
                except Exception as error:  # noqa: BLE001 - transport fault is diagnostic.
                    command_error = str(error)
        self._publish_diagnostics(
            state=feedback.state,
            feedback_age_s=feedback.measurement_age_s,
            feedback_fresh=feedback.measured_state_fresh,
            feedback_updated=feedback.feedback_updated,
            command=command,
            command_submitted=command_submitted,
            feedback_error=feedback.feedback_error,
            command_error=command_error,
            boundary_error=None,
        )

    def _publish_state(self, state: RobotArmState) -> None:
        if state.measured_joint_positions is None:
            return
        stamp = self.get_clock().now().to_msg()
        message = JointState()
        message.header.stamp = stamp
        message.name = list(self._joint_names)
        message.position = list(state.measured_joint_positions.values)
        self._measured_joint_state_publisher.publish(message)
        if self._visualization_joint_state_publisher is not None:
            visualization_message = JointState()
            visualization_message.header.stamp = stamp
            visualization_message.name = list(self._joint_names) + [
                "left_gripper_joint"
            ]
            visualization_message.position = list(
                state.measured_joint_positions.values
            ) + [0.0]
            self._visualization_joint_state_publisher.publish(visualization_message)

    def _publish_diagnostics(
        self,
        state: Optional[RobotArmState],
        feedback_age_s: Optional[float],
        feedback_fresh: bool,
        feedback_updated: bool,
        command: Optional[RobotArmCommand],
        command_submitted: bool,
        feedback_error: Optional[str] = None,
        command_error: Optional[str] = None,
        boundary_error: Optional[str] = None,
    ) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "myarm/robot_arm"
        status.hardware_id = "myarm_m750"
        status.level, status.message = self._diagnostic_level_and_message(
            state=state,
            feedback_fresh=feedback_fresh,
            feedback_error=feedback_error,
            command_error=command_error,
            boundary_error=boundary_error,
        )
        values = [
            KeyValue(key="feedback_fresh", value=str(feedback_fresh)),
            KeyValue(key="feedback_age_s", value=self._format_number(feedback_age_s)),
            KeyValue(key="feedback_updated", value=str(feedback_updated)),
            KeyValue(key="setpoint_submitted", value=str(command_submitted)),
            KeyValue(key="feedback_error", value=feedback_error or ""),
            KeyValue(key="setpoint_error", value=command_error or ""),
        ]
        if state is not None:
            values.extend([
                KeyValue(key="source", value=state.source),
                KeyValue(key="is_connected", value=str(state.is_connected)),
                KeyValue(key="is_powered", value=str(state.is_powered)),
                KeyValue(key="is_moving", value=str(state.is_moving)),
                KeyValue(key="state_sequence", value=str(state.sequence)),
                KeyValue(
                    key="consecutive_error_count",
                    value=str(state.consecutive_error_count),
                ),
                KeyValue(
                    key="last_error_message",
                    value=state.last_error_message or "",
                ),
            ])
        if command is not None:
            values.extend([
                KeyValue(key="setpoint_sequence", value=str(command.sequence)),
                KeyValue(
                    key="setpoint_speed_scale",
                    value=self._format_number(command.speed_scale),
                ),
            ])
        if boundary_error is not None:
            values.append(KeyValue(key="detail", value=boundary_error))
        status.values = values
        message.status = [status]
        self._diagnostics_publisher.publish(message)

    @staticmethod
    def _diagnostic_level_and_message(
        state: Optional[RobotArmState],
        feedback_fresh: bool,
        feedback_error: Optional[str],
        command_error: Optional[str],
        boundary_error: Optional[str],
    ):
        if boundary_error is not None:
            return DiagnosticStatus.ERROR, "robot_driver_error"
        if feedback_error:
            return DiagnosticStatus.ERROR, "robot_feedback_error"
        if command_error:
            return DiagnosticStatus.ERROR, "robot_setpoint_error"
        if state is None or not state.is_connected:
            return DiagnosticStatus.ERROR, "robot_disconnected"
        if state.measured_joint_positions is None:
            return DiagnosticStatus.WARN, "waiting_for_measured_state"
        if not feedback_fresh:
            return DiagnosticStatus.WARN, "measured_state_stale"
        if state.is_powered is False:
            return DiagnosticStatus.WARN, "robot_powered_off"
        return DiagnosticStatus.OK, "robot_state_fresh"

    def _publish_boundary_error(self, summary: str, detail: str) -> None:
        self._publish_diagnostics(
            state=self._safe_state(),
            feedback_age_s=None,
            feedback_fresh=False,
            feedback_updated=False,
            command=None,
            command_submitted=False,
            boundary_error=f"{summary}: {detail}",
        )

    def _stop_callback(self, request, response):
        del request
        self._clear_pending_setpoint()
        return self._run_lifecycle_callback("stop", self._service.stop, response)

    def _power_on_callback(self, request, response):
        del request
        return self._run_lifecycle_callback("power_on", self._service.power_on, response)

    def _power_off_callback(self, request, response):
        del request
        self._clear_pending_setpoint()
        return self._run_lifecycle_callback("power_off", self._service.power_off, response)

    def _run_lifecycle_callback(self, operation: str, action, response):
        try:
            state = action()
        except Exception as error:  # noqa: BLE001 - errors go to ROS client/diagnostics.
            detail = f"{operation} failed: {error}"
            self.get_logger().error(detail)
            response.success = False
            response.message = detail
            self._publish_boundary_error("lifecycle_error", detail)
            return response
        response.success = True
        response.message = f"{operation} accepted; state sequence {state.sequence}"
        self._publish_state(state)
        return response

    def _safe_state(self) -> Optional[RobotArmState]:
        try:
            return self._service.state
        except Exception:  # noqa: BLE001 - preserve original diagnostic failure.
            return None

    def destroy_node(self):
        """Release the backend transport without changing robot power state."""
        self._clear_pending_setpoint()
        try:
            self._service.disconnect()
        except Exception as error:  # noqa: BLE001 - shutdown remains best effort.
            self.get_logger().error(f"robot disconnect during shutdown failed: {error}")
        return super().destroy_node()

    @staticmethod
    def _format_number(value: Any) -> str:
        if value is None:
            return ""
        number = float(value)
        return "nan" if not math.isfinite(number) else f"{number:.9g}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MyArmRobotDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
