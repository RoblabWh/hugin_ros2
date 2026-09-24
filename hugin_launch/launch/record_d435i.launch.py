from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    DEFAULT_CONTAINER_NAME = "record_d435i"
    D435I_PATH = "/realsense/d435i"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "container_name", default_value=DEFAULT_CONTAINER_NAME
            ),
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                name=LaunchConfiguration("container_name"),
                namespace="",
                condition=IfCondition(
                    EqualsSubstitution(
                        LaunchConfiguration("container_name"), DEFAULT_CONTAINER_NAME
                    )
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("hugin_launch"),
                            "launch",
                            "sensor_d435i.launch.py",
                        ]
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("hugin_launch"),
                            "launch",
                            "record_bag.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "outbag": LaunchConfiguration("outbag"),
                    "topics": str(
                        [
                            f"{D435I_PATH}/color/image_raw",
                            f"{D435I_PATH}/color/metadata",
                            f"{D435I_PATH}/infra1/image_rect_raw",
                            f"{D435I_PATH}/infra1/metadata",
                            f"{D435I_PATH}/infra2/image_rect_raw",
                            f"{D435I_PATH}/infra2/metadata",
                            f"{D435I_PATH}/imu",
                        ]
                    ),
                }.items(),
            ),
        ]
    )
