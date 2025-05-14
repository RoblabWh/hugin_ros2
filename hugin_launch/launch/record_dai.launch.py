from launch import LaunchDescription
from launch.actions import OpaqueFunction
import hugin_launch.generators as gen
import hugin_launch.configurations as cfg


def generate_launch_description():
    return LaunchDescription(
        cfg.dai_source
        + cfg.dai_recorder
        + [
            OpaqueFunction(
                function=lambda context: gen.compose(
                    "dai_record_container",
                    gen.dai_source(context) + gen.dai_recorder(context),
                )
            )
        ]
    )
