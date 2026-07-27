import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateDemo(Node):
    def __init__(self):
        super().__init__("myarm_joint_state_demo")
        self._publisher = self.create_publisher(JointState, "joint_states", 10)
        self._start_time = self.get_clock().now()
        self.create_timer(0.1, self._publish)

    def _publish(self):
        seconds = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint",
            "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint",
            "left_gripper_joint",
        ]
        message.position = [
            0.5 * math.sin(seconds * 0.5),
            0.4 * math.sin(seconds * 0.4),
            0.5 * math.sin(seconds * 0.3),
            0.6 * math.sin(seconds * 0.6),
            0.4 * math.sin(seconds * 0.5),
            0.7 * math.sin(seconds * 0.7),
            0.015 * (1.0 + math.sin(seconds)),
        ]
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = JointStateDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
