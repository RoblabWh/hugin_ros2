from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from .utils import boolean, integer_list


def compose(name: str, nodes: list[ComposableNode]) -> list[ComposableNodeContainer]:
    return [
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name=name,
            namespace="",
            composable_node_descriptions=nodes,
            emulate_tty=True,
            output="screen",
        )
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


def bag_source(context) -> list[ComposableNode]:
    return [
        ComposableNode(
            package="rosbag2_transport",
            plugin="rosbag2_transport::Player",
            name="player",
            parameters=[
                {
                    "play.loop": boolean(LaunchConfiguration("loop").perform(context)),
                    "play.disable_keyboard_controls": True,
                    "storage.uri": LaunchConfiguration("input").perform(context),
                    "storage.max_cache_size": 2**32,
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


def dm_vio_nodes(context) -> list[ComposableNode]:
    vio_nodes = list()
    for cam_id in integer_list(LaunchConfiguration("cameras").perform(context)):
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
                        "skip_delay_visual_only": int(
                            LaunchConfiguration("cam_hz").perform(context)
                        ),
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
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )
    return vio_nodes


def dai_recorder(context) -> list[ComposableNode]:
    cam_hz = int(LaunchConfiguration("cam_hz").perform(context))
    imu_hz = int(LaunchConfiguration("imu_hz").perform(context))

    record_topics = list()
    if cam_hz > 0:
        for cam_id in integer_list(LaunchConfiguration("cameras").perform(context)):
            record_topics.append(f"/cam{cam_id}/image_raw")
            record_topics.append(f"/cam{cam_id}/image_info")
    if imu_hz > 0:
        record_topics.append("/imu/data_raw")

    return [
        ComposableNode(
            package="rosbag2_transport",
            plugin="rosbag2_transport::Recorder",
            name="recorder",
            parameters=[
                {
                    "record.topics": record_topics,
                    "record.is_discovery_disabled": True,
                    "record.disable_keyboard_controls": True,
                    "storage.uri": LaunchConfiguration("output"),
                    "storage.max_cache_size": 2**32,
                    # TODO test different presets
                    # "storage.storage_preset_profile": "zstd_small",
                }
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
    ]


#################
# Regular Nodes #
#################


def kalman_filter(context) -> list[Node]:
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
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="kalman_filter",
            parameters=[
                ekf_inputs,
                {
                    "world_frame": LaunchConfiguration("frame_odom").perform(context),
                    "odom_frame": LaunchConfiguration("frame_odom").perform(context),
                    "base_link_frame": LaunchConfiguration("frame_base").perform(
                        context
                    ),
                },
            ],
            remappings=[
                ("odometry/filtered", "odometry"),
            ],
            output="screen",
        )
    ]
