import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CommandJointStatePublisher(Node):
    """Demo bridge from a commanded six-axis pose to RViz joint state."""

    def __init__(self):
        super().__init__("myarm_command_joint_state_publisher")
        self._publisher = self.create_publisher(
            msg_type=JointState,
            topic="/joint_states",
            qos_profile=10,
        )
        self._subscriber = self.create_subscription(
            msg_type=JointState,
            topic="/myarm/command_joint_state",
            callback=self._command_callback,
            qos_profile=10,
        )
        self._joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
            "left_gripper_joint",
        ]
        self._joint_state_position = [0.0] * 6
        self.get_logger().info(
            "Bridging /myarm/command_joint_state to /joint_states for RViz demo."
        )

    def _command_callback(self, command: JointState):
        if len(command.position) != 6:
            self.get_logger().error(
                "Expected six arm positions, received {}. Command ignored.".format(
                    len(command.position)
                )
            )
            return

        self._joint_state_position = list(command.position)
        self.get_logger().info(
            "Received command: {}".format(self._joint_state_position)
        )
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = self._joint_names
        message.position = self._joint_state_position + [0.0]

        self._publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CommandJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
