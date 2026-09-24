from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    EqualsSubstitution,
)
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    DEFAULT_CONTAINER_NAME = "record_bag"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "container_name", default_value=DEFAULT_CONTAINER_NAME
            ),
            DeclareLaunchArgument(
                "topics",
                description="List of topics to record",
            ),
            DeclareLaunchArgument(
                "outbag",
                description="Path to output bag",
            ),
            # Increase default value by a lot to give rosbag enough time to write cached data to disk
            DeclareLaunchArgument(
                "sigterm_timeout",
                default_value=str(60 * 60),
                description="Seconds to wait until sigterm is sent",
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
            LoadComposableNodes(
                target_container=LaunchConfiguration("container_name"),
                composable_node_descriptions=[
                    ComposableNode(
                        package="rosbag2_transport",
                        plugin="rosbag2_transport::Recorder",
                        name="recorder",
                        parameters=[
                            {
                                "record.topics": LaunchConfiguration("topics"),
                                "record.disable_keyboard_controls": True,
                                "storage.uri": LaunchConfiguration("outbag"),
                                "storage.max_cache_size": 2**32,
                            }
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
        ]
    )
