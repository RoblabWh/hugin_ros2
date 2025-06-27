from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
import hugin_launch.configurations as cfg
import hugin_launch.generators as gen


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("cameras", default_value="0,1,2,3"),
        ]
        + cfg.mavros
        + cfg.dai_source
        + cfg.sensor_recorder
        + gen.gt_trigger_dummy()
        + gen.mavros()
        + [
            OpaqueFunction(
                function=lambda context: gen.compose(
                    "sensor_record_container",
                    gen.dai_source(context) + gen.sensor_recorder(context),
                )
            )
        ]
    )
