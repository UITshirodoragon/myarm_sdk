from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    description_share = Path(get_package_share_directory("myarm_description"))
    robot_description = (description_share / "urdf" / "myarm_m750_poe_v3_2.urdf").read_text()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="myarm_joint_state_publisher",
            executable="command_joint_state_publisher",
            output="screen",
        ),
    ])
