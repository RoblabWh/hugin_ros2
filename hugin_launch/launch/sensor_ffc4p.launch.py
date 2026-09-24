from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
    NotSubstitution,
    AndSubstitution,
)
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from math import radians


def generate_launch_description():
    DEFAULT_CONTAINER_NAME = "sensor_ffc4p"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "container_name", default_value=DEFAULT_CONTAINER_NAME
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("device_id", default_value="1944301001C1652E00"),
            DeclareLaunchArgument("usb_speed", default_value="4"),
            DeclareLaunchArgument("imu.hz", default_value="0"),
            DeclareLaunchArgument("cams", default_value="[0, 1, 2, 3]"),
            DeclareLaunchArgument("cam.hz", default_value="20.0"),
            DeclareLaunchArgument("cam.width", default_value="1920"),
            DeclareLaunchArgument("cam.height", default_value="1198"),
            DeclareLaunchArgument("cam.exposure", default_value="0"),
            DeclareLaunchArgument("cam.color", default_value="true"),
            DeclareLaunchArgument("cam.warmup", default_value="60"),
            DeclareLaunchArgument("sync.cams", default_value="[0, 1, 2, 3]"),
            DeclareLaunchArgument("sync.type", default_value="2"),
            DeclareLaunchArgument("sync.on_host", default_value="true"),
            DeclareLaunchArgument("exposure_in_frame_id", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_ffc4p",
                arguments=[
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "ffc4p_link",
                    # NOTE transformation from CAD
                    "--x",
                    str(-0.045042),
                    "--y",
                    str(0.000922),
                    "--z",
                    str(0.01202),
                    "--roll",
                    str(radians(180)),
                ],
                output="screen",
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
                        package="dai_vi_ros2",
                        plugin="dai_vi::ROS2Wrapper",
                        name="ffc4p",
                        namespace="luxonis",
                        parameters=[
                            {
                                "device_id": LaunchConfiguration("device_id"),
                                "usb_speed": LaunchConfiguration("usb_speed"),
                                "imu.hz": LaunchConfiguration("imu.hz"),
                                "cams": LaunchConfiguration("cams"),
                                "cam.hz": LaunchConfiguration("cam.hz"),
                                "cam.width": LaunchConfiguration("cam.width"),
                                "cam.height": LaunchConfiguration("cam.height"),
                                "cam.exposure": LaunchConfiguration("cam.exposure"),
                                "cam.color": LaunchConfiguration("cam.color"),
                                "cam.warmup": LaunchConfiguration("cam.warmup"),
                                "sync.cams": LaunchConfiguration("sync.cams"),
                                "sync.type": LaunchConfiguration("sync.type"),
                                "sync.on_host": LaunchConfiguration("sync.on_host"),
                                "exposure_in_frame_id": LaunchConfiguration(
                                    "exposure_in_frame_id"
                                ),
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
