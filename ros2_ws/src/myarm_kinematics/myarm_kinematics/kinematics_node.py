"""ROS 2 boundary for the MyArm M750 KinematicsService."""

import math
from typing import Any, Mapping

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from myarm_sdk.core import Pose, load_sdk_yaml
from myarm_sdk.service import KinematicsService, KinematicsServiceError
from rclpy.node import Node
from sensor_msgs.msg import JointState


class MyArmKinematicsNode(Node):
    """Receive TCP targets and publish a verified joint target at 5 Hz."""

    _SERVICES_CONFIG = "service/config/services.yaml"

    def __init__(self) -> None:
        super().__init__("myarm_kinematics")
        services_config = load_sdk_yaml(self._SERVICES_CONFIG)
        self._service_config = self._kinematics_config(services_config)
        self._topics = self._mapping(self._service_config["topics"], "kinematics topics")

        self._service = KinematicsService.from_config(
            service_config=self._service_config,
            package_share_directory=get_package_share_directory,
        )

        self._joint_target_publisher = self.create_publisher(
            JointState, str(self._topics["joint_target"]), 10
        )
        self._tcp_pose_publisher = self.create_publisher(
            PoseStamped, str(self._topics["tcp_pose"]), 10
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray, str(self._topics["diagnostics"]), 10
        )
        self._target_subscription = self.create_subscription(
            PoseStamped,
            str(self._topics["target_pose"]),
            self._target_pose_callback,
            10,
        )

        update_rate_hz = float(self._service_config["update_rate_hz"])
        if update_rate_hz <= 0.0:
            raise ValueError("kinematics update_rate_hz must be positive")
        self.create_timer(1.0 / update_rate_hz, self._step_and_publish)
        self.get_logger().info(
            f"myarm_kinematics is running at {update_rate_hz} Hz using {self._SERVICES_CONFIG}."
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
            self._publish_status(DiagnosticStatus.ERROR, "Invalid TCP target", str(error))
            self.get_logger().warning(f"Target rejected: {error}")

    def _step_and_publish(self) -> None:
        try:
            step = self._service.step()
        except (KinematicsServiceError, ValueError) as error:
            self._service.clear_target_pose()
            self._publish_status(DiagnosticStatus.ERROR, "IK failed", str(error))
            self.get_logger().warning(f"IK failed: {error}")
            return
        except Exception as error:  # noqa: BLE001 - keep unexpected backend faults visible in ROS.
            self._publish_status(DiagnosticStatus.ERROR, "Kinematics backend error", str(error))
            self.get_logger().error(f"Kinematics backend error: {error}")
            return

        self._publish_joint_target(step.joint_positions.values)
        self._publish_tcp_pose(step.tcp_pose)
        state = "tracking target" if step.target_active else "publishing initial pose"
        self._publish_status(DiagnosticStatus.OK, state, "")

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
        quaternion_norm = math.sqrt(
            orientation.x ** 2
            + orientation.y ** 2
            + orientation.z ** 2
            + orientation.w ** 2
        )
        if quaternion_norm < 1e-12:
            raise ValueError("target orientation quaternion must not have zero length")
        return Pose(
            position=(position.x, position.y, position.z),
            orientation=(orientation.x, orientation.y, orientation.z, orientation.w),
        )

    def _publish_joint_target(self, positions) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._service.joint_names)
        message.position = list(positions)
        self._joint_target_publisher.publish(message)

    def _publish_tcp_pose(self, pose: Pose) -> None:
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
        self._tcp_pose_publisher.publish(message)

    def _publish_status(self, level: int, summary: str, detail: str) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "myarm/kinematics"
        status.message = summary
        status.hardware_id = "myarm_m750"
        status.values = [
            KeyValue(key="base_frame", value=self._service.base_frame),
            KeyValue(key="detail", value=detail),
        ]
        message.status = [status]
        self._diagnostics_publisher.publish(message)


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
