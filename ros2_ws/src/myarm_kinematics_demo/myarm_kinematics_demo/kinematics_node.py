"""ROS boundary for MyArm M750 target-pose IK and FK reporting."""

import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState

from myarm_sdk.adapters.kinematics import (
    InverseKinematicsError,
    PinocchioKinematics,
)
from myarm_sdk.model import JointPositions, Pose


class KinematicsNode(Node):
    """Subscribe to target poses and publish IK commands plus verified FK."""

    _URDF_FILENAME = "myarm_m750_poe_v3_2.urdf"
    _BASE_FRAME = "base_link"
    _ARM_JOINT_NAMES = list(PinocchioKinematics.ARM_JOINT_NAMES)

    def __init__(self) -> None:
        super().__init__("myarm_kinematics")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("max_iterations", 100)
        self.declare_parameter("position_tolerance_m", 0.001)
        self.declare_parameter("orientation_tolerance_rad", 0.02)
        self.declare_parameter("damping", 0.001)
        self.declare_parameter("step_size", 0.5)

        description_share = Path(get_package_share_directory("myarm_description"))
        urdf_path = description_share / "urdf" / self._URDF_FILENAME
        self._kinematics = PinocchioKinematics(
            urdf_path=urdf_path,
            tool_frame=str(self.get_parameter("tool_frame").value),
            max_iterations=int(self.get_parameter("max_iterations").value),
            position_tolerance_m=float(
                self.get_parameter("position_tolerance_m").value
            ),
            orientation_tolerance_rad=float(
                self.get_parameter("orientation_tolerance_rad").value
            ),
            damping=float(self.get_parameter("damping").value),
            step_size=float(self.get_parameter("step_size").value),
        )
        self._last_solution = JointPositions((0.0,) * 6)

        self._command_publisher = self.create_publisher(
            JointState, "/myarm/command_joint_state", 10
        )
        self._current_pose_publisher = self.create_publisher(
            PoseStamped, "/myarm/kinematics/current_pose", 10
        )
        self._status_publisher = self.create_publisher(
            DiagnosticArray, "/myarm/kinematics/ik_status", 10
        )
        self._target_subscription = self.create_subscription(
            PoseStamped,
            "/myarm/command/target_pose",
            self._target_pose_callback,
            10,
        )
        self.get_logger().info(
            "Ready: target poses must be expressed in base_link; URDF loaded from "
            "myarm_description/urdf/{}.".format(self._URDF_FILENAME)
        )

    def _target_pose_callback(self, message: PoseStamped) -> None:
        try:
            target = self._sdk_pose_from_message(message)
            solution = self._kinematics.inverse(target, self._last_solution)
            current_pose = self._kinematics.forward(solution)
        except (InverseKinematicsError, ValueError) as error:
            self._publish_status(DiagnosticStatus.ERROR, "IK failed", str(error))
            self.get_logger().warning("IK target rejected: {}".format(error))
            return
        except Exception as error:
            self._publish_status(DiagnosticStatus.ERROR, "Kinematics backend error", str(error))
            self.get_logger().error("Kinematics backend error: {}".format(error))
            return

        self._last_solution = solution
        self._publish_command(solution)
        self._publish_current_pose(current_pose)
        self._publish_status(DiagnosticStatus.OK, "IK converged", "")

    def _sdk_pose_from_message(self, message: PoseStamped) -> Pose:
        if message.header.frame_id != self._BASE_FRAME:
            raise ValueError(
                "target frame must be '{}', received '{}'".format(
                    self._BASE_FRAME, message.header.frame_id
                )
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

    def _publish_command(self, solution: JointPositions) -> None:
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = self._ARM_JOINT_NAMES
        command.position = list(solution.values)
        self._command_publisher.publish(command)

    def _publish_current_pose(self, pose: Pose) -> None:
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._BASE_FRAME
        message.pose.position.x, message.pose.position.y, message.pose.position.z = pose.position
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = pose.orientation
        self._current_pose_publisher.publish(message)

    def _publish_status(self, level: int, summary: str, detail: str) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.level = level
        status.name = "myarm/kinematics/ik"
        status.message = summary
        status.hardware_id = "myarm_m750"
        status.values = [
            KeyValue(key="base_frame", value=self._BASE_FRAME),
            KeyValue(key="tool_frame", value=str(self.get_parameter("tool_frame").value)),
            KeyValue(key="detail", value=detail),
        ]
        message.status = [status]
        self._status_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
