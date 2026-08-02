"""Run the sequential NeuGrasp scan flow against FakeRobotArm only."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start fake feedback, scene and the selected sequential scan path.

    The coordinator is idle until a `/neugrasp/scan_workspace` action goal is
    sent.  This launch is deliberately not a physical robot profile.
    """
    share = Path(get_package_share_directory("neugrasp_bringup"))
    rviz_share = Path(get_package_share_directory("myarm_rviz2"))
    scan_config = LaunchConfiguration("scan_config")
    scene_config = LaunchConfiguration("scene_config")
    camera_calibration = LaunchConfiguration("camera_calibration")
    services_config = LaunchConfiguration("services_config")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    return LaunchDescription([
        DeclareLaunchArgument(
            "services_config", default_value="service/config/services.yaml"
        ),
        DeclareLaunchArgument(
            "scene_config", default_value=str(share / "config" / "neugrasp_scene.yaml")
        ),
        DeclareLaunchArgument(
            "scan_config", default_value=str(share / "config" / "neugrasp_scan_profiles.yaml")
        ),
        DeclareLaunchArgument(
            "camera_calibration",
            default_value=str(share / "config" / "neugrasp_fake_wrist_camera_calibration.yaml"),
        ),
        DeclareLaunchArgument("start_rviz", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(rviz_share / "config" / "neugrasp.rviz"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "neugrasp_system.launch.py")),
            launch_arguments={
                "enable_driver": "true",
                "enable_kinematics": "false",
                "enable_motion_execution": "true",
                "enable_cartesian_trajectory": "false",
                "enable_cartesian_execution": "true",
                "required_robot_arm_plugin_adapter": "fake_robot_arm",
                "services_config": services_config,
                "scene_config": scene_config,
                "use_wrist_camera": "true",
                "camera_calibration": camera_calibration,
            }.items(),
        ),
        Node(
            package="myarm_neugrasp",
            executable="neugrasp_scan_node",
            name="neugrasp_scan",
            parameters=[{
                "scan_config": scan_config,
                "services_config": services_config,
            }],
            output="screen",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(rviz_share / "launch" / "neugrasp_rviz_remote.launch.py")
            ),
            launch_arguments={"rviz_config": rviz_config}.items(),
            condition=IfCondition(start_rviz),
        ),
    ])
