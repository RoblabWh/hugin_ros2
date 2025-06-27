from launch import LaunchDescriptionEntity
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from .utils import boolean, integer_list, ShutdownClean, ShutdownFailure
from .configurations import log_level
from os import mkdir


def compose(
    name: str, nodes: list[ComposableNode], tty: bool = True, output: str = "screen"
) -> list[LaunchDescriptionEntity]:
    return [
        log_level,
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name=name,
            namespace="",
            composable_node_descriptions=nodes,
            ros_arguments=["--log-level", LaunchConfiguration("log_level")],
            emulate_tty=tty,
            output=output,
            on_exit=ShutdownFailure(reason="Component container exited unexpectedly"),
        ),
    ]


####################
# Composable Nodes #
####################


def dai_source(context) -> list[ComposableNode]:
    return [
        ComposableNode(
            package="dai_vi_ros2",
            plugin="dai_vi::ROS2Wrapper",
            name="dai_vi_sensor",
            parameters=[
                {
                    "camera_ids": integer_list(
                        LaunchConfiguration("cameras").perform(context)
                    ),
                    "cam_hz": int(LaunchConfiguration("cam_hz").perform(context)),
                    "imu_hz": int(LaunchConfiguration("imu_hz").perform(context)),
                    "frame_imu": LaunchConfiguration("frame_imu").perform(context),
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


def bag_source(context, remappings=None) -> list[ComposableNode]:
    return [
        ComposableNode(
            package="rosbag2_transport",
            plugin="rosbag2_transport::Player",
            name="player",
            parameters=[
                {
                    "play.loop": boolean(LaunchConfiguration("loop").perform(context)),
                    "play.disable_keyboard_controls": True,
                    "play.clock_publish_on_topic_publish": True,
                    "storage.uri": LaunchConfiguration("input").perform(context),
                    "storage.max_cache_size": 2**32,
                }
            ],
            remappings=remappings,
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


def dm_vio_nodes(context) -> list[ComposableNode]:
    vio_nodes = list()
    for cam_id in integer_list(LaunchConfiguration("cameras").perform(context)):
        results_base_path = LaunchConfiguration("results_path").perform(context)
        try:
            mkdir(results_base_path)
        except FileExistsError:
            pass
        vio_nodes.append(
            ComposableNode(
                package="dm_vio_ros2",
                plugin="dmvio::ROS2Wrapper",
                name=f"dm_vio_cam{cam_id}",
                parameters=[
                    {
                        "calibration": LaunchConfiguration("calibration").perform(
                            context
                        ),
                        "camera_index": cam_id,
                        # DM-VIO
                        "mode": int(LaunchConfiguration("mode").perform(context)),
                        "preset": int(LaunchConfiguration("preset").perform(context)),
                        "use_imu": boolean(
                            LaunchConfiguration("enable_imu").perform(context)
                        ),
                        "use_exposure": boolean(
                            LaunchConfiguration("enable_exposure").perform(context)
                        ),
                        "use_image_info": boolean(
                            LaunchConfiguration("use_image_info").perform(context)
                        ),
                        "quiet": boolean(LaunchConfiguration("quiet").perform(context)),
                        "nolog": boolean(LaunchConfiguration("nolog").perform(context)),
                        "max_skip_visual_init": int(
                            LaunchConfiguration("max_skip_visual_init").perform(context)
                        ),
                        "max_skip_visual_only": int(
                            LaunchConfiguration("max_skip_visual_only").perform(context)
                        ),
                        "max_skip_visual_inertial": int(
                            LaunchConfiguration("max_skip_visual_inertial").perform(
                                context
                            )
                        ),
                        "max_skip_full_reset": int(
                            LaunchConfiguration("max_skip_full_reset").perform(context)
                        ),
                        "imu_noise_factor": float(
                            LaunchConfiguration("imu_noise_factor").perform(context)
                        ),
                        "imu_bias_factor": float(
                            LaunchConfiguration("imu_bias_factor").perform(context)
                        ),
                        "skip_delay_visual_only": int(
                            LaunchConfiguration("cam_hz").perform(context)
                        ),
                        "results_path": PathJoinSubstitution(
                            [results_base_path, f"cam{cam_id}"]
                        ).perform(context),
                        # TF
                        "frame_origin": LaunchConfiguration("frame_odom").perform(
                            context
                        ),
                        "frame_odom": f"{
                            LaunchConfiguration('frame_odom').perform(context)
                        }{cam_id}",
                        "frame_base": LaunchConfiguration("frame_base").perform(
                            context
                        ),
                        "frame_imu": LaunchConfiguration("frame_imu").perform(context),
                        "frame_camera": f"cam{cam_id}_optical_frame",
                        "publish_tf": boolean(
                            LaunchConfiguration("publish_tf").perform(context)
                        ),
                        "update_origin": boolean(
                            LaunchConfiguration("update_origin").perform(context)
                        ),
                        "covariance_linear": float(
                            LaunchConfiguration("cov_lin").perform(context)
                        ),
                        "covariance_angular": float(
                            LaunchConfiguration("cov_ang").perform(context)
                        ),
                    }
                ],
                remappings=[
                    ("imu", "imu/data_raw"),
                    ("image_raw", f"cam{cam_id}/image_raw"),
                    ("image_info", f"cam{cam_id}/image_info"),
                    ("odometry", f"cam{cam_id}/odometry"),
                    ("pose_dso", f"cam{cam_id}/pose_dso"),
                    ("pose_dmvio", f"cam{cam_id}/pose_dmvio"),
                    ("tracking_state", f"cam{cam_id}/tracking_state"),
                    ("image_live", f"cam{cam_id}/image_live"),
                    ("image_depth", f"cam{cam_id}/image_depth"),
                    ("image_depth_float", f"cam{cam_id}/image_depth_float"),
                    ("camera_info", f"cam{cam_id}/camera_info"),
                    ("keyframes", f"cam{cam_id}/keyframes"),
                    ("pointscloud", f"cam{cam_id}/pointscloud"),
                    ("constraints", f"cam{cam_id}/constraints"),
                    ("trajectory", f"cam{cam_id}/trajectory"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
    return vio_nodes


def _data_recorder(topics: list[str], context) -> list[ComposableNode]:
    return [
        ComposableNode(
            package="rosbag2_transport",
            plugin="rosbag2_transport::Recorder",
            name="recorder",
            parameters=[
                {
                    "record.topics": topics,
                    "record.is_discovery_disabled": True,
                    "record.disable_keyboard_controls": True,
                    "storage.uri": LaunchConfiguration("output").perform(context),
                    "storage.max_cache_size": 2**32,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


def sensor_recorder(context) -> list[ComposableNode]:
    topics = list()

    for cam_id in integer_list(LaunchConfiguration("cameras").perform(context)):
        topics.append(f"/cam{cam_id}/image_raw")
        topics.append(f"/cam{cam_id}/image_info")
    topics.append("imu/data_raw")

    topics.append("/mavros/imu/data")  # Orientation from i.e. compass
    topics.append(
        "/mavros/imu/data_raw"
    )  # Angular velocity and linear acceleration from first IMU
    topics.append("/mavros/global_position/raw/fix")  # GPS data

    # TODO add mocap topic if available
    topics.append("/gt/trigger")

    return _data_recorder(topics, context)


def odometry_recorder(context) -> list[ComposableNode]:
    topics = list()

    for cam_id in integer_list(LaunchConfiguration("cameras").perform(context)):
        topics.append(f"/cam{cam_id}/odometry")
    topics.append("odometry")
    topics.append("navsat/odometry")
    topics.append("/tf_static")

    return _data_recorder(topics, context)


#################
# Regular Nodes #
#################


def kalman_filter(context) -> list[LaunchDescriptionEntity]:
    ekf_inputs = {}
    for i, cam_id in enumerate(
        integer_list(LaunchConfiguration("cameras").perform(context))
    ):
        sensor_name = f"odom{i}"
        sensor_topic = f"cam{cam_id}/odometry"
        ekf_inputs[sensor_name] = sensor_topic
        ekf_inputs[f"{sensor_name}_config"] = [
            # Pos
            True,
            True,
            True,
            True,
            True,
            True,
            # Vel
            False,
            False,
            False,
            False,
            False,
            False,
            # Acc
            False,
            False,
            False,
            False,
            False,
            False,
        ]
    return [
        log_level,
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="kalman_filter",
            parameters=[
                {
                    "frequency": 20.0,
                    "sensor_timeout": 0.5,
                    # TODO for debugging
                    # "print_diagnostics": True,
                    # "debug": True,
                    "smooth_lagged_data": True,
                    "history_length": 5.0,
                    "use_sim_time": boolean(
                        LaunchConfiguration("use_sim_time").perform(context)
                    ),
                },
                {
                    "world_frame": LaunchConfiguration("frame_odom").perform(context),
                    "odom_frame": LaunchConfiguration("frame_odom").perform(context),
                    "base_link_frame": LaunchConfiguration("frame_base").perform(
                        context
                    ),
                },
                ekf_inputs,
            ],
            remappings=[
                ("odometry/filtered", "odometry"),
            ],
            ros_arguments=["--log-level", LaunchConfiguration("log_level")],
            output="screen",
        ),
    ]


def navsat_transform() -> list[LaunchDescriptionEntity]:
    return [
        log_level,
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            parameters=[
                {
                    "frequency": 5.0,
                    # NOTE WH Google Maps Coords in https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
                    "magnetic_declination_radians": 0.027663468644110123,
                    "yaw_offset": 0.0,
                }
            ],
            remappings=[
                ("gps/fix", "/mavros/global_position/raw/fix"),
                ("imu/data", "/mavros/imu/data"),
                ("odometry/filtered", "odometry"),
                ("odometry/gps", "navsat/odometry"),
            ],
            ros_arguments=["--log-level", LaunchConfiguration("log_level")],
            output="screen",
        ),
    ]


###################
# Launch Includes #
###################


def mavros() -> list[LaunchDescriptionEntity]:
    return [
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
        )
    ]


##########
# Helper #
##########


def shutdown_on_bag_end(timeout: float = 1.0) -> list[LaunchDescriptionEntity]:
    return [
        Node(
            package="hugin_launch",
            executable="wait_for_bag_end",
            name="wait_for_bag_end",
            parameters=[{"timeout": timeout}],
            output="both",
            on_exit=ShutdownClean(reason="Bag finished"),
        )
    ]


def gt_trigger_dummy() -> list[LaunchDescriptionEntity]:
    return [
        Node(
            package="hugin_launch",
            executable="dummy_publisher",
            name="gt_trigger_dummy",
            remappings=[("dummy", "/gt/trigger")],
            output="both",
        )
    ]
