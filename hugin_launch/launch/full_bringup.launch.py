from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from hugin_launch.configurations import calibration, cameras
from hugin_launch.utils import DeclareLaunchArgumentWithNewDefault


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "name", default_value="hugin", description="Name of the robot"
            ),
            DeclareLaunchArgumentWithNewDefault(
                calibration,
                PathJoinSubstitution(
                    [
                        FindPackageShare("hugin_launch"),
                        "config",
                        "calibration.json",
                    ]
                ),
            ),
            DeclareLaunchArgumentWithNewDefault(cameras, "0,1,2,3"),
            GroupAction(
                [
                    PushRosNamespace(LaunchConfiguration("name")),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("hugin_launch"),
                                    "launch",
                                    "online_vio.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "calibration": LaunchConfiguration("calibration"),
                            "cameras": LaunchConfiguration("cameras"),
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("hugin_launch"),
                                    "launch",
                                    "mavros.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "fcu_url": "/dev/ttyTHS1:230400",
                        }.items(),
                    ),
                ]
            ),
        ]
    )
