from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import hugin_launch.generators as gen
import hugin_launch.configurations as cfg


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("cameras", default_value="0,1"),
            DeclareLaunchArgument("calibration", default_value=PathJoinSubstitution([FindPackageShare("hugin_launch"), "config", "calibration_tumvi.json"])),
            DeclareLaunchArgument("use_image_info", default_value="false"),
            DeclareLaunchArgument("imu_tf_type", default_value="tumvi"),
        ]
        + cfg.bag_source
        + cfg.dm_vio_nodes
        + cfg.kalman_filter
        + cfg.odometry_recorder
        + [
            OpaqueFunction(
                function=lambda context: gen.kalman_filter(context)
                + gen.compose(
                    "vio_container",
                    gen.dm_vio_nodes(context)
                    + gen.odometry_recorder(context)
                    + gen.bag_source(context, [
                        ("imu0", "imu/data_raw"),
                        ("vrpn_client/raw_transform", "gt")
                    ]),
                    output="log",
                )
            ),
        ]
        + gen.shutdown_on_bag_end()
    )
