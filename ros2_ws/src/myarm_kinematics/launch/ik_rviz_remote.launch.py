"""Legacy kinematics-only launch; use myarm_bringup for a full system."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start no TF or joint-state bridge, avoiding duplicate publishers."""
    return LaunchDescription([
        Node(
            package="myarm_kinematics",
            executable="kinematics_node",
            name="myarm_kinematics",
            output="screen",
        ),
    ])
