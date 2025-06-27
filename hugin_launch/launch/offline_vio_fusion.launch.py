from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
import hugin_launch.generators as gen
import hugin_launch.configurations as cfg


def generate_launch_description():
    return LaunchDescription(
        [DeclareLaunchArgument("use_sim_time", default_value="true")]
        + cfg.bag_source
        + cfg.dm_vio_nodes
        + cfg.kalman_filter
        + [
            OpaqueFunction(
                function=lambda context: gen.compose(
                    "vio_container", gen.dm_vio_nodes(context) + gen.bag_source(context)
                )
                + gen.kalman_filter(context)
            )
        ]
    )
