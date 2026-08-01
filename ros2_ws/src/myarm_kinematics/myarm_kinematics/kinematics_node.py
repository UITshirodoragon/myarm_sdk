"""ROS 2 boundary for measured-state-seeded MyArm M750 kinematics."""

import math
import time
from typing import Any, Mapping

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from myarm_sdk.core import JointPositions, Pose, load_sdk_yaml
from myarm_sdk.service import KinematicsService
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class MyArmKinematicsNode(Node):
    """Map ROS pose/feedback topics to the SDK kinematics service at 5 Hz."""

    _SERVICES_CONFIG = "service/config/services.yaml"

    def __init__(self) -> None:
        super().__init__("myarm_kinematics")
        services_config = load_sdk_yaml(self._SERVICES_CONFIG)
        self._service_config = self._kinematics_config(services_config)
        self._robot_config = self._mapping(services_config.get("robot"), "robot")
        self._topics = self._mapping(self._service_config["topics"], "kinematics topics")
        self._service = KinematicsService.from_config(
            service_config=self._service_config,
            package_share_directory=get_package_share_directory,
            robot_config=self._robot_config,
        )

        self._joint_goal_publisher = self.create_publisher(
            JointState, str(self._topics["joint_goal"]), 10
        )
        self._current_tcp_pose_publisher = self.create_publisher(
            PoseStamped, str(self._topics["current_tcp_pose"]), 10
        )
        self._commanded_tcp_pose_publisher = self.create_publisher(
            PoseStamped, str(self._topics["commanded_tcp_pose"]), 10
        )
        self._ik_status_publisher = self.create_publisher(
            DiagnosticArray, str(self._topics["ik_status"]), 10
        )
        self._target_subscription = self.create_subscription(
            PoseStamped,
            str(self._topics["target_pose"]),
            self._target_pose_callback,
            10,
        )
        self._measured_joint_subscription = self.create_subscription(
            JointState,
            str(self._topics["measured_joint_state"]),
            self._measured_joint_state_callback,
            qos_profile_sensor_data,
        )

        update_rate_hz = float(self._service_config["update_rate_hz"])
        if update_rate_hz <= 0.0:
            raise ValueError("kinematics update_rate_hz must be positive")
        self.create_timer(1.0 / update_rate_hz, self._step_and_publish)
        self.get_logger().info(
            "myarm_kinematics is running at {} Hz. It expects canonical model-space "
            "feedback on {}.".format(
                update_rate_hz, self._topics["measured_joint_state"]
            )
        )

    @staticmethod
    def _mapping(value: Any, name: str) -> Mapping[str, Any]:
        if not isinstance(value, dict):
            raise TypeError(f"{name} must be a mapping")
        return value

    def _kinematics_config(
        self, services_config: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        services = self._mapping(services_config.get("services"), "services")
        kinematics = self._mapping(services.get("kinematics"), "services.kinematics")
        if not bool(kinematics.get("enabled", False)):
            raise RuntimeError("Kinematics service is disabled in services.yaml")
        return kinematics

    def _target_pose_callback(self, message: PoseStamped) -> None:
        try:
            self._service.set_target_pose(self._sdk_pose_from_message(message))
        except ValueError as error:
            self._publish_boundary_error("invalid_target", str(error))
            self.get_logger().warning(f"Target rejected: {error}")

    def _measured_joint_state_callback(self, message: JointState) -> None:
        try:
            joints = self._canonical_joint_positions_from_message(message)
            self._service.update_measured_joint_positions(
                joints, received_at_monotonic_s=time.monotonic()
            )
        except ValueError as error:
            self._publish_boundary_error("invalid_measured_joint_state", str(error))
            self.get_logger().error(f"Measured joint state rejected: {error}")

    def _step_and_publish(self) -> None:
        try:
            step = self._service.step(now_monotonic_s=time.monotonic())
        except Exception as error:  # noqa: BLE001 - backend failures must remain visible in ROS.
            self._publish_boundary_error("kinematics_backend_error", str(error))
            self.get_logger().error(f"Kinematics backend error: {error}")
            return

        if step.command_updated:
            self._publish_joint_goal(step.commanded_joint_positions)
            self._publish_pose(
                self._commanded_tcp_pose_publisher, step.commanded_tcp_pose
            )
        if step.measured_tcp_pose is not None:
            self._publish_pose(self._current_tcp_pose_publisher, step.measured_tcp_pose)
        if step.ik_result is not None and not step.ik_result.converged:
            reason = (
                step.ik_result.failure_reason.value
                if step.ik_result.failure_reason is not None
                else "unknown"
            )
            self.get_logger().warning(
                f"IK command rejected ({reason}): {step.ik_result.detail}"
            )
        self._publish_step_status(step)

    def _canonical_joint_positions_from_message(self, message: JointState) -> JointPositions:
        if message.name:
            if len(message.name) != len(message.position):
                raise ValueError("measured JointState name and position lengths differ")
            if len(set(message.name)) != len(message.name):
                raise ValueError("measured JointState contains duplicate joint names")
            positions_by_name = dict(zip(message.name, message.position))
            missing = [
                name for name in self._service.joint_names if name not in positions_by_name
            ]
            if missing:
                raise ValueError(
                    "measured JointState is missing arm joints: {}".format(
                        ", ".join(missing)
                    )
                )
            return JointPositions(
                tuple(positions_by_name[name] for name in self._service.joint_names)
            )
        if len(message.position) != len(self._service.joint_names):
            raise ValueError(
                "unnamed measured JointState must contain exactly six canonical arm positions"
            )
        return JointPositions(message.position)

    def _sdk_pose_from_message(self, message: PoseStamped) -> Pose:
        if message.header.frame_id != self._service.base_frame:
            raise ValueError(
                f"target frame must be '{self._service.base_frame}', received '{message.header.frame_id}'"
            )
        position = message.pose.position
        orientation = message.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("target pose contains non-finite values")
        return Pose(
            position=(position.x, position.y, position.z),
            orientation=(orientation.x, orientation.y, orientation.z, orientation.w),
        )

    def _publish_joint_goal(self, joints: JointPositions) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._service.joint_names)
        message.position = list(joints.values)
        self._joint_goal_publisher.publish(message)

    def _publish_pose(self, publisher, pose: Pose) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._service.base_frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = pose.orientation
        publisher.publish(message)

    def _publish_step_status(self, step) -> None:
        result = step.ik_result
        if result is None:
            level = (
                DiagnosticStatus.OK
                if step.measured_state_fresh
                else DiagnosticStatus.WARN
            )
            summary = (
                "waiting for target"
                if step.measured_state_fresh
                else "waiting for fresh measured joint state"
            )
            detail = ""
        elif result.converged:
            level = (
                DiagnosticStatus.WARN
                if result.singularity.near_singular
                else DiagnosticStatus.OK
            )
            summary = "ik_converged"
            detail = result.detail
        else:
            level = DiagnosticStatus.ERROR
            reason = result.failure_reason.value if result.failure_reason else "unknown"
            summary = f"ik_failed:{reason}"
            detail = result.detail

        values = [
            KeyValue(key="base_frame", value=self._service.base_frame),
            KeyValue(key="tool_frame", value=self._service.tool_frame),
            KeyValue(key="measured_state_fresh", value=str(step.measured_state_fresh)),
            KeyValue(
                key="measured_state_age_s",
                value=self._format_number(step.measured_state_age_s),
            ),
            KeyValue(
                key="seed_source",
                value=step.seed_source.value if step.seed_source is not None else "",
            ),
            KeyValue(key="detail", value=detail),
        ]
        if result is not None:
            values.extend([
                KeyValue(key="converged", value=str(result.converged)),
                KeyValue(
                    key="failure_reason",
                    value=(result.failure_reason.value if result.failure_reason else ""),
                ),
                KeyValue(
                    key="position_residual_m",
                    value=self._format_number(result.position_residual_m),
                ),
                KeyValue(
                    key="orientation_residual_rad",
                    value=self._format_number(result.orientation_residual_rad),
                ),
                KeyValue(key="iteration_count", value=str(result.iteration_count)),
                KeyValue(
                    key="minimum_singular_value",
                    value=self._format_number(
                        result.singularity.minimum_singular_value
                    ),
                ),
                KeyValue(
                    key="condition_number",
                    value=self._format_number(result.singularity.condition_number),
                ),
                KeyValue(
                    key="jacobian_rank",
                    value=str(result.singularity.rank),
                ),
                KeyValue(
                    key="near_singular",
                    value=str(result.singularity.near_singular),
                ),
                KeyValue(
                    key="singular",
                    value=str(result.singularity.singular),
                ),
                KeyValue(
                    key="active_joint_limits",
                    value=",".join(result.active_joint_limits),
                ),
                KeyValue(
                    key="minimum_joint_limit_margin_rad",
                    value=self._format_number(
                        result.minimum_joint_limit_margin_rad
                    ),
                ),
            ])

        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "myarm/kinematics/ik"
        status.message = summary
        status.hardware_id = "myarm_m750"
        status.values = values
        message.status = [status]
        self._ik_status_publisher.publish(message)

    def _publish_boundary_error(self, summary: str, detail: str) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = DiagnosticStatus.ERROR
        status.name = "myarm/kinematics/ik"
        status.message = summary
        status.hardware_id = "myarm_m750"
        status.values = [KeyValue(key="detail", value=detail)]
        message.status = [status]
        self._ik_status_publisher.publish(message)

    @staticmethod
    def _format_number(value) -> str:
        if value is None:
            return ""
        value = float(value)
        return "nan" if not math.isfinite(value) else f"{value:.9g}"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MyArmKinematicsNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
