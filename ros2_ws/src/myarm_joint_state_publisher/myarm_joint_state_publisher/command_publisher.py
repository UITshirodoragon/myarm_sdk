import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CommandJointStatePublisher(Node):
    """Bridge a commanded six-axis target to the joint state used by RViz."""

    def __init__(self):
        super().__init__("myarm_command_joint_state_publisher")
        self._publisher = self.create_publisher(
            msg_type=JointState,
            topic="/joint_states",
            qos_profile=10,
        )
        self._subscriber = self.create_subscription(
            msg_type=JointState,
            topic="/myarm/command/joint_target",
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
        self.create_timer(0.2, self._publish)
        self.get_logger().info(
            "Bridging /myarm/command/joint_target to /joint_states at 5 Hz for RViz."
        )

    def _command_callback(self, command: JointState):
        if len(command.position) != 6:
            self.get_logger().error(
                f"Expected six arm positions, received {len(command.position)}. Command ignored."
            )
            return

        self._joint_state_position = list(command.position)
        self.get_logger().info(
            f"Received command: {self._joint_state_position}"
        )

    def _publish(self):
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
