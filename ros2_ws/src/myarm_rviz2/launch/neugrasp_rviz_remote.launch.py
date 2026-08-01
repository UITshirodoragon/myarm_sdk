"""Start an RViz-only Neugrasp viewer on a remote host."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start RViz only; the robot-side host owns TF and robot state."""
    default_config = (
        Path(get_package_share_directory("myarm_rviz2"))
        / "config"
        / "neugrasp.rviz"
    )
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(default_config),
            description="Absolute path to the RViz configuration to load.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time for RViz when the robot-side graph provides /clock.",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
