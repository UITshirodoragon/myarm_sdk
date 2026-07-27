from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start RViz2; robot_state_publisher must already be running."""
    config = Path(get_package_share_directory("myarm_rviz2")) / "config" / "myarm_m750.rviz"
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(config)],
            output="screen",
        ),
    ])
