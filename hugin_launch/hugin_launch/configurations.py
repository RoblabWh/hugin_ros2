from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from .utils import radians

####################
# Common Arguments #
####################

log_level = DeclareLaunchArgument(
    "log_level",
    default_value="info",
    description="Log level",
)

## Sensors
cameras = DeclareLaunchArgument(
    "cameras",
    default_value="0",
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
    "calibration", description="Path to basalt calibration.json"
)

## TF2
publish_tf = DeclareLaunchArgument(
    "publish_tf", default_value="true", description="Publish TF"
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


tf_base_imu = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_imu",
    arguments=[
        #NOTE translation from f3d
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
)

tf_base_fcu = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_fcu",
    arguments=[
        #NOTE translation from f3d
        #NOTE tf to center of fcu (never needed)
        # "--z",
        # "0.0025",
        #NOTE tf to first IMU on fcu (only needed tf so publish this)
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
)

tf_base_gps = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_gps",
    arguments=[
        #NOTE translation from f3d
        "--z",
        "0.1",
        "--frame-id",
        LaunchConfiguration("frame_base"),
        "--child-frame-id",
        LaunchConfiguration("frame_gps"),
    ],
    output="screen",
)

######################
### Configurations ###
######################

dai_source = [
    cameras,
    cam_hz,
    imu_hz,
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
    DeclareLaunchArgument("mode", default_value="0", description="DM-VIO mode"),
    DeclareLaunchArgument(
        "preset",
        # TODO maybe set to 1 for realtime
        default_value="0",
        description="DM-VIO preset",
    ),
    DeclareLaunchArgument("enable_imu", default_value="true", description="Enable IMU"),
    DeclareLaunchArgument(
        "quiet", default_value="true", description="DM-VIO disable console output"
    ),
    DeclareLaunchArgument(
        "nolog", default_value="true", description="DM-VIO disable logging"
    ),
    DeclareLaunchArgument("max_skip_visual_init", default_value="0", description="Maximum number of frames to skip during visual initialization"),
    DeclareLaunchArgument("max_skip_visual_only", default_value="1", description="Maximum number of frames to skip during visual only mode"),
    DeclareLaunchArgument("max_skip_visual_inertial", default_value="2", description="Maximum number of frames to skip during visual inertial mode"),
    DeclareLaunchArgument("max_skip_full_reset", default_value="-1", description="Maximum number of frames to skip on full reset"),
    frame_odom,
    frame_base,
    frame_imu,
    publish_tf,
    DeclareLaunchArgument("update_origin", default_value="false", description="Trigger update of origin on initialization"),
    tf_base_imu,
]

dai_recorder = [
    cameras,
    cam_hz,
    imu_hz,
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

kalman_filter = [
    cameras,
    frame_odom,
    frame_base,
]
