from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "calibration",
                description="Path to the basalt calibration.json describing the rig.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("cairn"), "config", "cairn.yaml"]
                ),
                description="Parameter file, the calibration_file is overridden by the launch argument.",
            ),
            ComposableNodeContainer(
                name="cairn_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    ComposableNode(
                        package="cairn",
                        plugin="cairn::Tracker",
                        name="cairn",
                        parameters=[
                            LaunchConfiguration("params_file"),
                            {"calibration_file": LaunchConfiguration("calibration")},
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
                output="screen",
            ),
        ]
    )
