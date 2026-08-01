"""Start an RViz-only Neugrasp viewer on a remote host."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("myarm_rviz2")) / "config" / "neugrasp.rviz"
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(config)],
            output="screen",
        ),
    ])
