from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import hugin_launch.configurations as cfg


def generate_launch_description():
    return LaunchDescription(
        cfg.mavros
        + cfg.tf_base_fcu
        + cfg.tf_base_gps
        + [
            cfg.log_level,
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
                remappings=[
                    ("/mavros/odometry/out", "/odometry"),
                    ("/mavros/odometry/in", "/mavros/odometry/unused"),
                    ("/mavros/local_position/odom", "/mavros/odometry"),
                ],
                ros_arguments=["--log-level", LaunchConfiguration("log_level")],
                output="both",
            ),
        ]
    )
