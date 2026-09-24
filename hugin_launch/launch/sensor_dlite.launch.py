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
    DEFAULT_CONTAINER_NAME = "sensor_dlite"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "container_name", default_value=DEFAULT_CONTAINER_NAME
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("device_id", default_value="18443010319FA61200"),
            DeclareLaunchArgument("usb_speed", default_value="4"),
            DeclareLaunchArgument("imu.hz", default_value="0"),
            DeclareLaunchArgument("cams", default_value="[0, 1, 2]"),
            DeclareLaunchArgument("cam_stereo.hz", default_value="40.0"),
            DeclareLaunchArgument("cam_stereo.width", default_value="0"),
            DeclareLaunchArgument("cam_stereo.height", default_value="0"),
            DeclareLaunchArgument("cam_stereo.warmup", default_value="40"),
            DeclareLaunchArgument("cam_color.hz", default_value="20.0"),
            DeclareLaunchArgument("cam_color.width", default_value="1920"),
            DeclareLaunchArgument("cam_color.height", default_value="1200"),
            DeclareLaunchArgument("cam_color.color", default_value="true"),
            DeclareLaunchArgument("cam_color.warmup", default_value="20"),
            DeclareLaunchArgument("sync.cams", default_value="[1, 2]"),
            DeclareLaunchArgument("sync.type", default_value="3"),
            DeclareLaunchArgument("sync.on_host", default_value="true"),
            DeclareLaunchArgument("exposure_in_frame_id", default_value="true"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_dlite",
                arguments=[
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "dlite_link",
                    # TODO transformation eyeballed
                    "--x",
                    str(0.15),
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
                        name="dlite",
                        namespace="luxonis",
                        parameters=[
                            {
                                "device_id": LaunchConfiguration("device_id"),
                                "usb_speed": LaunchConfiguration("usb_speed"),
                                "imu.hz": LaunchConfiguration("imu.hz"),
                                "cams": LaunchConfiguration("cams"),
                                "cam.hz": LaunchConfiguration("cam_stereo.hz"),
                                "cam.width": LaunchConfiguration("cam_stereo.width"),
                                "cam.height": LaunchConfiguration("cam_stereo.height"),
                                "cam.color": False,
                                "cam.warmup": LaunchConfiguration("cam_stereo.warmup"),
                                "cam0.hz": LaunchConfiguration("cam_color.hz"),
                                "cam0.warmup": LaunchConfiguration("cam_color.warmup"),
                                "cam0.color": LaunchConfiguration("cam_color.color"),
                                "cam0.width": LaunchConfiguration("cam_color.width"),
                                "cam0.height": LaunchConfiguration("cam_color.height"),
                                "sync.cams": LaunchConfiguration("sync.cams"),
                                "sync.type": LaunchConfiguration("sync.type"),
                                "sync.on_host": LaunchConfiguration("sync.on_host"),
                                "exposure_in_frame_id": LaunchConfiguration(
                                    "exposure_in_frame_id"
                                ),
                            }
                        ],
                        remappings=[
                            ("~/cam0/image_raw", "~/color/image_raw"),
                            ("~/cam0/metadata", "~/color/metadata"),
                            ("~/cam1/image_raw", "~/stereo1/image_raw"),
                            ("~/cam1/metadata", "~/stereo1/metadata"),
                            ("~/cam2/image_raw", "~/stereo2/image_raw"),
                            ("~/cam2/metadata", "~/stereo2/metadata"),
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
