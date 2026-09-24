from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from math import radians


def generate_launch_description():
    DEFAULT_CONTAINER_NAME = "stream_pano"
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
                            "virtcam_pano.launch.py",
                        ]
                    )
                )
            ),
            LoadComposableNodes(
                target_container=LaunchConfiguration("container_name"),
                composable_node_descriptions=[
                    ComposableNode(
                        package="image_gst",
                        plugin="image_gst::AppSrc",
                        name="pano_gst",
                        namespace="luxonis",
                        parameters=[
                            {
                                "pipeline": "appsrc max-buffers=2 block=false leaky-type=downstream ! "
                                "videoconvert ! video/x-raw,format=BGRx ! "
                                "nvvidconv ! "
                                "nvv4l2h265enc profile=Main preset-level=UltraFastPreset maxperf-enable=true insert-sps-pps=true insert-vui=true bitrate=5000000 iframeinterval=30 ! "
                                "rtph265pay aggregate-mode=zero-latency config-interval=-1 ! "
                                "udpsink sync=false async=false host=127.0.0.1 port=5602"
                            }
                        ],
                        remappings=[("image_raw", "pano/image_raw")],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
        ]
    )
