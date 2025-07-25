from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    EqualsSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from .utils import radians
from tempfile import gettempdir

####################
# Common Arguments #
####################

log_level = DeclareLaunchArgument(
    "log_level",
    default_value="info",
    description="Log level",
)

use_sim_time = DeclareLaunchArgument(
    "use_sim_time",
    default_value="false",
    description="Use simulation time",
)

## Sensors
cameras = DeclareLaunchArgument(
    "cameras",
    default_value="0,1,2,3",
    description="Comma separated list of camera ids",
)
cam_hz = DeclareLaunchArgument(
    "cam_hz",
    default_value="20",
    description="Camera frequency",
)
imu_hz = DeclareLaunchArgument(
    "imu_hz",
    default_value="200",
    description="IMU frequency",
)
calibration = DeclareLaunchArgument(
    "calibration",
    description="Path to basalt calibration.json",
    default_value=PathJoinSubstitution(
        [FindPackageShare("hugin_launch"), "config", "calibration.json"]
        # [FindPackageShare("hugin_launch"), "config", "calibration_imu_base.json"]
    ),
)
enable_image_info = DeclareLaunchArgument(
    "use_image_info", default_value="true", description="Use image info messages"
)
max_init_time = DeclareLaunchArgument(
    "max_time_between_init_frames",
    default_value="0.5",
    description="Maximum time between initialization frames in seconds",
)

## TF2
publish_tf = DeclareLaunchArgument(
    "publish_tf", default_value="false", description="Publish TF"
)
frame_odom = DeclareLaunchArgument(
    "frame_odom", default_value="odom", description="frame_id of VIO"
)
frame_base = DeclareLaunchArgument(
    "frame_base",
    default_value="base_link",
    description="frame_id of robot base link",
)
frame_imu = DeclareLaunchArgument(
    "frame_imu", default_value="imu", description="frame_id of IMU"
)
frame_fcu = DeclareLaunchArgument(
    "frame_fcu", default_value="fcu", description="frame_id of FCU"
)
frame_gps = DeclareLaunchArgument(
    "frame_gps", default_value="gps", description="frame_id of GPS"
)


##############
# Static TFs #
##############

tfs_base_imu = [
    DeclareLaunchArgument(
        "imu_tf_type", default_value="hugin", description="IMU TF type"
    ),
    frame_base,
    frame_imu,
    Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_imu",
        arguments=[
            # NOTE translation from f3d
            "--x",
            "0.02796",
            "--y",
            "-0.009015",
            "--z",
            "0.038742",
            "--roll",
            str(radians(180)),
            "--yaw",
            str(radians(90)),
            "--frame-id",
            LaunchConfiguration("frame_base"),
            "--child-frame-id",
            LaunchConfiguration("frame_imu"),
        ],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("imu_tf_type"), "hugin")
        ),
    ),
    Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_imu",
        arguments=[
            "--yaw",
            str(radians(90)),
            "--frame-id",
            LaunchConfiguration("frame_base"),
            "--child-frame-id",
            LaunchConfiguration("frame_imu"),
        ],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("imu_tf_type"), "tumvi")
        ),
    ),
    Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_imu",
        arguments=[
            "--roll",
            str(radians(180)),
            "--pitch",
            str(radians(-90)),
            "--frame-id",
            LaunchConfiguration("frame_base"),
            "--child-frame-id",
            LaunchConfiguration("frame_imu"),
        ],
        output="screen",
        condition=IfCondition(
            EqualsSubstitution(LaunchConfiguration("imu_tf_type"), "euroc")
        ),
    ),
]

tf_base_fcu = [
    frame_base,
    frame_fcu,
    Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_fcu",
        arguments=[
            # NOTE translations from f3d
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
            "--frame-id",
            LaunchConfiguration("frame_base"),
            "--child-frame-id",
            LaunchConfiguration("frame_fcu"),
        ],
        output="screen",
    ),
]

tf_base_gps = [
    frame_base,
    frame_gps,
    Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_gps",
        arguments=[
            # NOTE translation from f3d
            "--z",
            "0.1",
            "--frame-id",
            LaunchConfiguration("frame_base"),
            "--child-frame-id",
            LaunchConfiguration("frame_gps"),
        ],
        output="screen",
    ),
]

######################
### Configurations ###
######################

dai_source = [
    cameras,
    cam_hz,
    imu_hz,
    DeclareLaunchArgument(
        "exposure",
        default_value="0",
        description="Exposure time in microseconds, 0 for auto exposure",
    ),
    enable_image_info,
    frame_imu,
]

bag_source = [
    DeclareLaunchArgument(
        "input",
        description="Path to input bag",
    ),
    DeclareLaunchArgument(
        "loop",
        description="Play bag in loop",
        default_value="false",
    ),
]

dm_vio_nodes = [
    cameras,
    cam_hz,
    calibration,
    DeclareLaunchArgument(
        "projection",
        default_value="0.2",
        description="Output camera projection parameters",
    ),
    DeclareLaunchArgument(
        "resolution",
        default_value="512",
        description="Output camera resolution",
    ),
    DeclareLaunchArgument("mode", default_value="0", description="DM-VIO mode"),
    DeclareLaunchArgument(
        "preset",
        # Single threaded IMU Initialization
        default_value="0",
        # Multi threaded IMU Initialization (race condition on reset)
        # default_value="1",
        description="DM-VIO preset",
    ),
    DeclareLaunchArgument("enable_imu", default_value="true", description="Enable IMU"),
    DeclareLaunchArgument(
        "enable_exposure", default_value="true", description="Enable Exposure"
    ),
    DeclareLaunchArgument(
        "quiet", default_value="true", description="DM-VIO disable console output"
    ),
    DeclareLaunchArgument(
        "nolog", default_value="true", description="DM-VIO disable logging"
    ),
    DeclareLaunchArgument(
        "max_skip_visual_init",
        default_value="0",
        description="Maximum number of frames to skip during visual initialization",
    ),
    DeclareLaunchArgument(
        "max_skip_visual_only",
        default_value="1",
        description="Maximum number of frames to skip during visual only mode",
    ),
    DeclareLaunchArgument(
        "max_skip_visual_inertial",
        default_value="2",
        description="Maximum number of frames to skip during visual inertial mode",
    ),
    DeclareLaunchArgument(
        "max_skip_full_reset",
        default_value="-1",
        description="Maximum number of frames to skip on full reset",
    ),
    DeclareLaunchArgument(
        "results_path",
        default_value=PathJoinSubstitution([gettempdir(), "dm_vio_results"]),
        description="Path to store DM-VIO results",
    ),
    DeclareLaunchArgument(
        "imu_noise_factor",
        # default_value="160.0",
        # default_value="320.0",
        default_value="480.0",
        # default_value="640.0",
        description="Factor to inflate IMU noise by",
    ),
    DeclareLaunchArgument(
        "imu_bias_factor",
        # default_value="500.0",
        # default_value="1000.0",
        default_value="1500.0",
        # default_value="2000.0",
        description="Factor to inflate IMU bias by",
    ),
    DeclareLaunchArgument(
        "pgba_skip_first_kfs",
        default_value="1",
        description="Skip first KFs in PGBA initialization",
    ),
    DeclareLaunchArgument(
        "init_normalized_error_threshold",
        # indoor
        # default_value="0.2",
        # #outdoor
        default_value="1.0",
        description="Normalized error threshold for coarse initialization to trigger full reset",
    ),
    max_init_time,
    DeclareLaunchArgument(
        "start_skip",
        default_value="2",
        description="Number of frames to skip at the start",
    ),
    DeclareLaunchArgument(
        "candidate_points",
        default_value="0",
        description="Number of candidate points to use in DSO",
    ),
    DeclareLaunchArgument(
        "active_points",
        default_value="0",
        description="Number of active points to use in DSO",
    ),
    frame_odom,
    frame_base,
    frame_imu,
    publish_tf,
    DeclareLaunchArgument(
        "update_origin",
        default_value="false",
        description="Trigger update of origin on initialization",
    ),
    enable_image_info,
    DeclareLaunchArgument(
        "covariance_linear",
        default_value="0.1",
        # default_value="0.2",
        # default_value="0.5",
        # default_value="1.0",
        # default_value="10.0",
        description="Linear covariance for DM-VIO",
    ),
    DeclareLaunchArgument(
        "covariance_angular",
        default_value="0.1",
        description="Angular covariance for DM-VIO",
    ),
    DeclareLaunchArgument(
        "covariance_linear_scaling",
        default_value="0.0",
        description="Linear covariance scaling factor for DM-VIO",
    ),
    DeclareLaunchArgument(
        "covariance_angular_scaling",
        default_value="0.0",
        description="Angular covariance scaling factor for DM-VIO",
    ),
] + tfs_base_imu

_data_recorder = [
    DeclareLaunchArgument(
        "output",
        description="Path to output bag",
    ),
    # Increase default value by alot to give rosbag enough time to write cached data to disk
    DeclareLaunchArgument(
        "sigterm_timeout",
        default_value=str(60 * 60),
        description="Seconds to wait until sigterm is send",
    ),
]
sensor_recorder = _data_recorder + [cameras]
odometry_recorder = _data_recorder + [cameras]

kalman_filter = [
    DeclareLaunchArgument(
        "kalman_filter_type",
        default_value="ekf",
        description="Type of Kalman filter to use",
    ),
    DeclareLaunchArgument(
        "kalman_filter_mode",
        default_value="relative",
        description="Mode of Kalman filter to use",
    ),
    use_sim_time,
    cameras,
    cam_hz,
    max_init_time,
    frame_odom,
    frame_base,
] + tf_base_fcu

navsat_transform = [] + tf_base_gps

mavros = [
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
    frame_odom,
    frame_base,
    frame_fcu,
    frame_gps,
]
