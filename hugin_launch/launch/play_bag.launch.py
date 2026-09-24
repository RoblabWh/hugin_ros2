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
    DEFAULT_CONTAINER_NAME = "play_bag"
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "container_name", default_value=DEFAULT_CONTAINER_NAME
            ),
            DeclareLaunchArgument(
                "inbag",
                description="Path to input bag",
            ),
            DeclareLaunchArgument(
                "loop",
                default_value="false",
                description="Play bag in a loop",
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
                        plugin="rosbag2_transport::Player",
                        name="player",
                        parameters=[
                            {
                                "play.loop": LaunchConfiguration("loop"),
                                "play.disable_keyboard_controls": True,
                                "play.clock_publish_frequency": 1000.0,
                                "storage.uri": LaunchConfiguration("inbag"),
                                "storage.max_cache_size": 2**32,
                            }
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
            ),
        ]
    )
