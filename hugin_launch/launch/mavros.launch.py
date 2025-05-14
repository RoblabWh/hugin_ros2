from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fcu_url",
                #TODO adjust
                default_value="/dev/ttyTHS1:230400",
                description="FCU URL",
            ),
            DeclareLaunchArgument(
                "gcs_url",
                default_value="",
                description="GCS URL",
            ),
            DeclareLaunchArgument(
                "tgt_system",
                default_value="1",
                description="Target system ID",
            ),
            DeclareLaunchArgument(
                "tgt_component",
                default_value="1",
                description="Target component ID",
            ),
            DeclareLaunchArgument(
                "fcu_protocol",
                default_value="v2.0",
                description="FCU protocol version",
            ),
            Node(
                package="mavros",
                executable="mavros_node",
                parameters=[
                    {
                        "fcu_url": LaunchConfiguration("fcu_url"),
                        "gcs_url": LaunchConfiguration("gcs_url"),
                        "tgt_system": LaunchConfiguration("tgt_system"),
                        "tgt_component": LaunchConfiguration("tgt_component"),
                        "fcu_protocol": LaunchConfiguration("fcu_protocol"),
                        "plugin_allowlist": [
                            "sys_*",
                            "command",
                            "imu",
                        ],
                    },
                    PathJoinSubstitution(
                        [
                            FindPackageShare("hugin_launch"),
                            "config",
                            "mavros.yaml",
                        ]
                    ),
                ],
                output="screen",
            ),
        ]
    )
