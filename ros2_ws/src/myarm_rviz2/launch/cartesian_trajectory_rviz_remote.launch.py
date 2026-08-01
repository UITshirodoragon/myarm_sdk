"""Start a remote RViz viewer with the Cartesian reference-path display."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Run RViz only; TF and robot state remain owned by the robot-side host."""
    config = (
        Path(get_package_share_directory("myarm_rviz2"))
        / "config"
        / "myarm_m750.rviz"
    )
    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(config)],
            output="screen",
        ),
    ])
