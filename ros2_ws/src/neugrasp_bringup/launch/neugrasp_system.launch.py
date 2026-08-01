"""Bring up MyArm plus Neugrasp application frames on the robot-side host."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Run the sole robot_state_publisher and Neugrasp static scene TF owner."""
    neugrasp_share = Path(get_package_share_directory("neugrasp_bringup"))
    myarm_bringup_share = Path(get_package_share_directory("myarm_bringup"))

    enable_driver = LaunchConfiguration("enable_driver")
    enable_kinematics = LaunchConfiguration("enable_kinematics")
    enable_motion_execution = LaunchConfiguration("enable_motion_execution")
    enable_scene_frames = LaunchConfiguration("enable_scene_frames")
    enable_rviz = LaunchConfiguration("enable_rviz")
    use_wrist_camera = LaunchConfiguration("use_wrist_camera")
    wrist_camera_mount_xyz = LaunchConfiguration("wrist_camera_mount_xyz")
    wrist_camera_mount_rpy = LaunchConfiguration("wrist_camera_mount_rpy")
    wrist_camera_xyz = LaunchConfiguration("wrist_camera_xyz")
    wrist_camera_rpy = LaunchConfiguration("wrist_camera_rpy")
    scene_config = LaunchConfiguration("scene_config")

    application_xacro = PathJoinSubstitution([
        FindPackageShare("myarm_description"),
        "urdf",
        "myarm_m750_application.urdf.xacro",
    ])
    robot_description = Command([
        FindExecutable(name="xacro"), " ", application_xacro, " ",
        "use_wrist_camera:=", use_wrist_camera, " ",
        "wrist_camera_mount_xyz:='", wrist_camera_mount_xyz, "' ",
        "wrist_camera_mount_rpy:='", wrist_camera_mount_rpy, "' ",
        "wrist_camera_xyz:='", wrist_camera_xyz, "' ",
        "wrist_camera_rpy:='", wrist_camera_rpy, "'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("enable_driver", default_value="true"),
        DeclareLaunchArgument("enable_kinematics", default_value="true"),
        DeclareLaunchArgument("enable_motion_execution", default_value="true"),
        DeclareLaunchArgument("enable_scene_frames", default_value="true"),
        DeclareLaunchArgument("enable_rviz", default_value="false"),
        DeclareLaunchArgument("use_wrist_camera", default_value="false"),
        DeclareLaunchArgument("wrist_camera_mount_xyz", default_value="0 0 0"),
        DeclareLaunchArgument("wrist_camera_mount_rpy", default_value="0 0 0"),
        DeclareLaunchArgument("wrist_camera_xyz", default_value="0 0 0"),
        DeclareLaunchArgument("wrist_camera_rpy", default_value="0 0 0"),
        DeclareLaunchArgument(
            "scene_config",
            default_value=str(neugrasp_share / "config" / "neugrasp_scene.yaml"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(myarm_bringup_share / "launch" / "myarm_system.launch.py")
            ),
            launch_arguments={
                "enable_driver": enable_driver,
                "enable_kinematics": enable_kinematics,
                "enable_motion_execution": enable_motion_execution,
                # This application launch owns the only robot_state_publisher.
                "enable_robot_state_publisher": "false",
            }.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="neugrasp_bringup",
            executable="neugrasp_static_scene_frames_node",
            name="neugrasp_static_scene_frames",
            parameters=[{"scene_config": scene_config}],
            condition=IfCondition(enable_scene_frames),
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(neugrasp_share / "config" / "neugrasp.rviz")],
            condition=IfCondition(enable_rviz),
            output="screen",
        ),
    ])
