from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_vio_node(context, *args, **kwargs):
    cam_ids = [int(i) for i in LaunchConfiguration('camera_ids').perform(context).split(',')]

    vio_nodes = []
    for cam_id in cam_ids:
        vio_nodes.append(
            ComposableNode(
                package='dm_vio_ros2',
                plugin='dmvio::ROS2Wrapper',
                name=f'dmvio_cam{cam_id}',
                parameters=[{
                    'calibration': LaunchConfiguration('calibration_file').perform(context),
                    'camera_index': cam_id,
                    'frame_world': LaunchConfiguration('frame_world').perform(context),
                }],
                remappings= [
                        ('imu', 'imu/data_raw'),
                        ('image_raw', f'cam{cam_id}/image_raw'),
                        ('image_info', f'cam{cam_id}/image_info'),
                        ('pose_raw', f'cam{cam_id}/pose_raw'),
                        ('pose_metric', f'cam{cam_id}/pose_metric'),
                        ('pose_dmvio', f'cam{cam_id}/pose_dmvio'),
                        ('tracking_state', f'cam{cam_id}/tracking_state'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    return [ComposableNodeContainer(
            package='rclcpp_components',
            executable='component_container_mt',
            name='vio_container',
            namespace='',
            composable_node_descriptions=[
                ComposableNode(
                    package='dai_vi_ros2',
                    plugin='dai_vi::ROS2Wrapper',
                    name='dai_vi_sensor',
                    parameters=[{'camera_ids': cam_ids}],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ] + vio_nodes,
            emulate_tty=True,
            output='screen',
        )]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_ids',
             default_value='0',
             description='Comma separated list of camera ids'
             ),
        DeclareLaunchArgument(
            'calibration_file',
             description='Path to calibration file'
             ),
        DeclareLaunchArgument(
            'frame_world',
                default_value='world',
             description='frame_id of VIO'
             ),
        OpaqueFunction(function=generate_vio_node)
    ])
