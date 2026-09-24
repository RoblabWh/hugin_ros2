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
                default_value="/dev/ttyTHS2:460800",
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
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_fcu",
                arguments=[
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "fcu",
                    # NOTE transformation from CAD
                    # NOTE tf to center of fcu (never needed)
                    # "--z",
                    # "0.0025",
                    # NOTE tf to first IMU on fcu (only needed tf so publish this)
                    "--x",
                    "-0.003913",
                    "--y",
                    "0.011861",
                    "--z",
                    "-0.0033",
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_base_gps",
                arguments=[
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "gps",
                    # NOTE transformation from CAD
                    "--z",
                    "0.1",
                ],
                output="screen",
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
                            "global_position",
                            "local_position",
                            "odometry",
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
                output="both",
            ),
        ]
    )
