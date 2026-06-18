from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def _fov_params(cfg):
    """Return dict of FOV parameters from LaunchConfiguration objects."""
    return {
        'fov_enabled':    cfg['fov_enabled'],
        'fov_use_angular': cfg['fov_use_angular'],
        'fov_h_min_deg':  cfg['fov_h_min_deg'],
        'fov_h_max_deg':  cfg['fov_h_max_deg'],
        'fov_v_min_deg':  cfg['fov_v_min_deg'],
        'fov_v_max_deg':  cfg['fov_v_max_deg'],
    }


def generate_launch_description():
    cfg = {k: LaunchConfiguration(k) for k in (
        'fov_enabled', 'fov_use_angular',
        'fov_h_min_deg', 'fov_h_max_deg',
        'fov_v_min_deg', 'fov_v_max_deg',
    )}

    crop_nodes = [
        Node(
            package='multiespectral_acquire',
            executable='pointcloud_crop_node',
            name='pointcloud_crop',
            output='screen',
            parameters=[{'input_topic': 'ouster/points_sync', **_fov_params(cfg)}],
        ),
        Node(
            package='multiespectral_acquire',
            executable='image_crop_node',
            name='range_image_crop',
            output='screen',
            parameters=[{'input_topic': 'ouster/range_image_sync', **_fov_params(cfg)}],
        ),
        Node(
            package='multiespectral_acquire',
            executable='image_crop_node',
            name='reflec_image_crop',
            output='screen',
            parameters=[{'input_topic': 'ouster/reflec_image_sync', **_fov_params(cfg)}],
        ),
        Node(
            package='multiespectral_acquire',
            executable='image_crop_node',
            name='signal_image_crop',
            output='screen',
            parameters=[{'input_topic': 'ouster/signal_image_sync', **_fov_params(cfg)}],
        ),
        Node(
            package='multiespectral_acquire',
            executable='image_crop_node',
            name='nearir_image_crop',
            output='screen',
            parameters=[{'input_topic': 'ouster/nearir_image_sync', **_fov_params(cfg)}],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument('fov_enabled',    default_value='true'),
        DeclareLaunchArgument('fov_use_angular', default_value='true'),
        DeclareLaunchArgument('fov_h_min_deg',  default_value='-20.0'),
        DeclareLaunchArgument('fov_h_max_deg',  default_value='20.0'),
        DeclareLaunchArgument('fov_v_min_deg',  default_value='-15.0'),
        DeclareLaunchArgument('fov_v_max_deg',  default_value='15.0'),
        *crop_nodes,
    ])
