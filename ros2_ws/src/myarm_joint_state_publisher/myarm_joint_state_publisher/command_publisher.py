import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CommandJointStatePublisher(Node):
    """Legacy demo bridge from a commanded target to the RViz joint state.

    Do not run this node together with ``myarm_robot_driver``: the production
    driver is the sole publisher of actual-feedback ``/joint_states``.
    """

    def __init__(self):
        super().__init__("myarm_command_joint_state_publisher")
        self._publisher = self.create_publisher(
            msg_type=JointState,
            topic="/joint_states",
            qos_profile=10,
        )
        self._subscriber = self.create_subscription(
            msg_type=JointState,
            topic="/myarm/command/joint_goal",
            callback=self._command_callback,
            qos_profile=10,
        )
        self._arm_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_flex_joint",
            "forearm_roll_joint",
            "wrist_flex_joint",
            "wrist_roll_joint",
        ]
        self._joint_names = self._arm_joint_names + [
            "left_gripper_joint",
        ]
        self._joint_state_position = [0.0] * 6
        self.create_timer(0.2, self._publish)
        self.get_logger().info(
            "Legacy bridge active at 5 Hz. Do not run with myarm_robot_driver."
        )

    def _command_callback(self, command: JointState):
        if command.name:
            if len(command.name) != len(command.position):
                self.get_logger().error(
                    "Joint target name and position lengths differ. Command ignored."
                )
                return
            if len(set(command.name)) != len(command.name):
                self.get_logger().error("Joint target contains duplicate names. Command ignored.")
                return
            positions_by_name = dict(zip(command.name, command.position))
            missing = [
                name for name in self._arm_joint_names if name not in positions_by_name
            ]
            if missing:
                self.get_logger().error(
                    "Joint target is missing arm joints: {}. Command ignored.".format(
                        ", ".join(missing)
                    )
                )
                return
            self._joint_state_position = [
                positions_by_name[name] for name in self._arm_joint_names
            ]
        elif len(command.position) != 6:
            self.get_logger().error(
                f"Expected six arm positions, received {len(command.position)}. Command ignored."
            )
            return
        else:
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
