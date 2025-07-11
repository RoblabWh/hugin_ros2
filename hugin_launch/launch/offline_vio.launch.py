from launch import LaunchDescription
from launch.actions import OpaqueFunction
import hugin_launch.generators as gen
import hugin_launch.configurations as cfg


def generate_launch_description():
    return LaunchDescription(
        cfg.bag_source
        + cfg.dm_vio_nodes
        + [
            OpaqueFunction(
                function=lambda context: gen.compose(
                    "vio_container", gen.bag_source(context) + gen.dm_vio_nodes(context)
                )
            )
        ] + gen.reset_rviz()
    )
