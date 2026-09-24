from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
    NotSubstitution,
    AndSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from math import radians


def generate_launch_description():
    DEFAULT_CONTAINER_NAME = "sensor_d435i"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "container_name", default_value=DEFAULT_CONTAINER_NAME
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("enable_gyro", default_value="true"),
            DeclareLaunchArgument("enable_accel", default_value="true"),
            DeclareLaunchArgument("imu_fps", default_value="200"),
            DeclareLaunchArgument("enable_color", default_value="true"),
            DeclareLaunchArgument("color_stream", default_value="640x480x30"),
            DeclareLaunchArgument("color_format", default_value="BGR8"),
            DeclareLaunchArgument("enable_infra", default_value="true"),
            DeclareLaunchArgument("infra_stream", default_value="640x480x30"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_d435i",
                arguments=[
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "d435i_link",
                    # NOTE transformation from CAD
                    "--x",
                    str(0.16),
                    "--y",
                    str(0.0175),
                    "--z",
                    str(-0.012),
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_d435i_imu",
                arguments=[
                    "--frame-id",
                    "d435i_link",
                    "--child-frame-id",
                    "d435i_imu_optical_frame",
                    # NOTE transformation from URDF
                    "--x",
                    str(-0.01174),
                    "--y",
                    str(-0.00552),
                    "--z",
                    str(0.0051),
                    "--roll",
                    str(radians(-90)),
                    "--yaw",
                    str(radians(-90)),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_d435i_color",
                arguments=[
                    "--frame-id",
                    "d435i_link",
                    "--child-frame-id",
                    "d435i_color_optical_frame",
                    # NOTE transformation from URDF
                    "--x",
                    str(-0.000612617),
                    "--y",
                    str(0.014717762),
                    "--z",
                    str(0.000200338),
                    "--roll",
                    str(radians(-90)),
                    "--yaw",
                    str(radians(-90)),
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("use_sim_time")),
            ),
            ComposableNodeContainer(
                package="rclcpp_components",
                executable="component_container_mt",
                name=LaunchConfiguration("container_name"),
                namespace="",
                condition=IfCondition(
                    AndSubstitution(
                        EqualsSubstitution(
                            LaunchConfiguration("container_name"),
                            DEFAULT_CONTAINER_NAME,
                        ),
                        NotSubstitution(LaunchConfiguration("use_sim_time")),
                    ),
                ),
            ),
            LoadComposableNodes(
                target_container=LaunchConfiguration("container_name"),
                composable_node_descriptions=[
                    ComposableNode(
                        package="realsense2_camera",
                        plugin="realsense2_camera::RealSenseNodeFactory",
                        name="d435i",
                        namespace="realsense",
                        parameters=[
                            {
                                "camera_name": "d435i",
                                "camera_namespace": "realsense",
                                "enable_infra1": LaunchConfiguration("enable_infra"),
                                "enable_infra2": LaunchConfiguration("enable_infra"),
                                "enable_depth": False,
                                "depth_module.infra_profile": LaunchConfiguration(
                                    "infra_stream"
                                ),
                                "depth_module.emitter_enabled": 0,
                                "enable_color": LaunchConfiguration("enable_color"),
                                "rgb_camera.color_profile": LaunchConfiguration(
                                    "color_stream"
                                ),
                                "rgb_camera.color_format": LaunchConfiguration(
                                    "color_format"
                                ),
                                "enable_gyro": LaunchConfiguration("enable_gyro"),
                                "enable_accel": LaunchConfiguration("enable_accel"),
                                "gyro_fps": LaunchConfiguration("imu_fps"),
                                "accel_fps": LaunchConfiguration("imu_fps"),
                                "unite_imu_method": 1,
                            }
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
                condition=IfCondition(
                    NotSubstitution(LaunchConfiguration("use_sim_time"))
                ),
            ),
        ]
    )
