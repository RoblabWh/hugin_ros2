from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from math import radians


def generate_launch_description():
    DEFAULT_CONTAINER_NAME = "virtcam_fpv"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "container_name", default_value=DEFAULT_CONTAINER_NAME
            ),
            DeclareLaunchArgument("sensor_bringup", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_fpv",
                arguments=[
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "fpv_optical",
                    "--x",
                    "0.15",
                    "--yaw",
                    str(radians(-90)),
                    "--roll",
                    str(radians(-90)),
                ],
                output="screen",
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
                            "sensor_ffc4p.launch.py",
                        ]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("sensor_bringup")),
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
                ),
                launch_arguments={
                    "enable_color": "false",
                    "enable_infra": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("sensor_bringup")),
            ),
            LoadComposableNodes(
                target_container=LaunchConfiguration("container_name"),
                composable_node_descriptions=[
                    ComposableNode(
                        package="virtcam",
                        plugin="virtcam::PerspectiveCamera",
                        name="fpv",
                        namespace="luxonis",
                        parameters=[
                            {
                                "input.calibration": PathJoinSubstitution(
                                    [
                                        FindPackageShare("hugin_launch"),
                                        "config",
                                        "calibration_ffc4p_d435i.json",
                                    ]
                                ),
                                "input.index": 2,
                                "input.depth": 100.0,
                                "input.frame_id": "d435i_imu_optical_frame",
                                "output.frame_id": "fpv_optical",
                            }
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
        ]
    )
