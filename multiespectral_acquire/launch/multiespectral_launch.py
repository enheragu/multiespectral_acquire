from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('multiespectral_acquire')

    session_folder  = LaunchConfiguration('session_folder')
    output_path     = LaunchConfiguration('output_path')
    frame_rate      = LaunchConfiguration('output_frame_rate')
    main_trigger_topic = LaunchConfiguration('main_trigger_topic')

    return LaunchDescription([
        # ---- Arguments ----
        DeclareLaunchArgument('session_folder',     default_value='test_session'),
        DeclareLaunchArgument('dataset_output_path', default_value='/tmp/multiespectral_data'),
        DeclareLaunchArgument('output_path',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('dataset_output_path'),
                PathJoinSubstitution(['mult_', session_folder]),
            ])),
        DeclareLaunchArgument('output_frame_rate',  default_value='1'),
        DeclareLaunchArgument('gnss_topic',         default_value='/gnss/fix'),
        DeclareLaunchArgument('odom_topic',         default_value='/odometry/combined'),
        DeclareLaunchArgument('main_trigger_topic', default_value='visible_camera/image_with_metadata'),

        # Camera PTP / exposure. lwir_use_ptp=true is safe: the adapter verifies
        # an actual PTP lock and falls back to software calibration otherwise
        # (see cameras_only.launch.py).
        DeclareLaunchArgument('visible_use_ptp',    default_value='false'),
        DeclareLaunchArgument('lwir_use_ptp',       default_value='true'),
        DeclareLaunchArgument('ouster_exposure_ns', default_value='30000'),

        # LIDAR FOV
        DeclareLaunchArgument('fov_enabled',        default_value='true'),
        DeclareLaunchArgument('fov_use_angular',    default_value='true'),
        # See capture_sync.launch.py — H bounds = measured stripe-free region
        # (old [-8.5,19] hit the destagger ragged edge → black lines).
        DeclareLaunchArgument('fov_h_min_deg',      default_value='-13.5'),
        DeclareLaunchArgument('fov_h_max_deg',      default_value='14.0'),
        DeclareLaunchArgument('fov_v_min_deg',      default_value='-16.0'),
        DeclareLaunchArgument('fov_v_max_deg',      default_value='7.0'),

        # ---- All nodes inside /Multiespectral namespace ----
        GroupAction(actions=[
            PushRosNamespace('Multiespectral'),

            # Visible camera (Basler)
            Node(
                package='multiespectral_acquire',
                executable='basler_camera_handler',
                name='visible_camera',
                output='screen',
                respawn=True,
                respawn_delay=15.0,
                parameters=[{
                    'image_topic':      'visible_camera',
                    'camera_ip':        EnvironmentVariable('MULTIESPECTRAL_VISIBLE_IP', default_value=''),
                    'output_frame_rate': ParameterValue(frame_rate, value_type=float),
                    'camera_info_url':  'package://multiespectral_acquire/conf/visible_params.yaml',
                    'dataset_name':     session_folder,
                    'use_ptp':          LaunchConfiguration('visible_use_ptp'),
                }],
            ),

            # LWIR camera (FLIR)
            Node(
                package='multiespectral_acquire',
                executable='flir_camera_handler',
                name='lwir_camera',
                output='screen',
                respawn=True,
                respawn_delay=10.0,
                parameters=[{
                    'image_topic':      'lwir_camera',
                    'camera_ip':        EnvironmentVariable(
                        'MULTIESPECTRAL_LWIR_IP', default_value='192.168.4.6'),
                    'output_frame_rate': 30.0,
                    'camera_info_url':  'package://multiespectral_acquire/conf/lwir_params.yaml',
                    'dataset_name':     session_folder,
                    'use_ptp':          LaunchConfiguration('lwir_use_ptp'),
                }],
            ),

            # LIDAR crop nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share, '/launch/multiespectral_lidar_crop.launch.py']),
                launch_arguments={
                    'fov_enabled':    LaunchConfiguration('fov_enabled'),
                    'fov_use_angular': LaunchConfiguration('fov_use_angular'),
                    'fov_h_min_deg':  LaunchConfiguration('fov_h_min_deg'),
                    'fov_h_max_deg':  LaunchConfiguration('fov_h_max_deg'),
                    'fov_v_min_deg':  LaunchConfiguration('fov_v_min_deg'),
                    'fov_v_max_deg':  LaunchConfiguration('fov_v_max_deg'),
                }.items(),
            ),

            # Buffer handlers
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share, '/launch/multiespectral_buffer_handlers.launch.py']),
                launch_arguments={
                    'output_path':       output_path,
                    'main_trigger_topic': main_trigger_topic,
                    'ouster_exposure_ns': LaunchConfiguration('ouster_exposure_ns'),
                    'gnss_topic':        LaunchConfiguration('gnss_topic'),
                    'odom_topic':        LaunchConfiguration('odom_topic'),
                    'main_trigger_hz':   frame_rate,
                }.items(),
            ),
        ]),
    ])
