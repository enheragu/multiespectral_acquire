"""Capture synchronization: lidar crop nodes + buffer compositor.

Counterpart of cameras_only.launch.py — run by hitos_sync.service after the
cameras and sensors are up. session_folder MUST match the one passed to
cameras_only.launch.py (hitos_cameras.service shares it via
/tmp/hitos_session.env) so files land in the same dataset session.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('multiespectral_acquire')

    session_folder  = LaunchConfiguration('session_folder')
    output_path     = LaunchConfiguration('output_path')
    frame_rate      = LaunchConfiguration('output_frame_rate')

    return LaunchDescription([
        # ---- Arguments (same structure/defaults as multiespectral_launch.py) ----
        DeclareLaunchArgument('session_folder',     default_value='test_session'),
        DeclareLaunchArgument('dataset_output_path', default_value='/tmp/multiespectral_data'),
        # Grouping dir: 'mult_' for execution datasets, 'calib_' for calibration
        # ones, so the two toolchains never mix. Set by hitos_sync.service per mode.
        DeclareLaunchArgument('session_prefix',     default_value='mult_'),
        DeclareLaunchArgument('output_path',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('dataset_output_path'),
                # Concatenate prefix+session into ONE dir name (mult_<date> /
                # calib_<date>), not PathJoin (which inserts a '/' → a bare
                # 'calib_' grouping dir + the date inside). A [a, b] list is
                # concatenated without a separator.
                [LaunchConfiguration('session_prefix'), session_folder],
            ])),
        DeclareLaunchArgument('output_frame_rate',  default_value='1'),
        DeclareLaunchArgument('calibration_mode',   default_value='false'),
        DeclareLaunchArgument('gnss_topic',         default_value='/gnss/fix'),
        DeclareLaunchArgument('odom_topic',         default_value='/odometry/combined'),
        DeclareLaunchArgument('main_trigger_topic', default_value='visible_camera/image_with_metadata'),
        DeclareLaunchArgument('ouster_exposure_ns', default_value='30000'),

        # LIDAR FOV
        DeclareLaunchArgument('fov_enabled',        default_value='true'),
        DeclareLaunchArgument('fov_use_angular',    default_value='true'),
        # H bounds = the measured stripe-free region for the cropped beams
        # (≥99% row fill in rings 48-80 → cols [451,531] = h[-21.4,+6.7]). The old
        # [-8.5,19] reached into the destagger's ragged right edge → black lines.
        # Centred ~-7° in this convention (≈ where the LiDAR's valid data sits);
        # whether that matches the camera forward is verified on the live image.
        # Centred ~forward within the (re-pointed) azimuth window's stripe-free
        # region h[-13,+19.3]. H[-13.5,17] = ~30° centred +1.75° (max raised +3° so
        # the images follow the cloud's wider right FOV — still inside the ~+19°
        # stripe-free edge); V[-16,10] = 26° (V shifted down; top raised +3° for a
        # bit more upward FOV — still trimmed vs the +45 sensor max for the
        # unmeasured lidar-camera pitch offset).
        # Iterating against the live image; proper fix = measure the extrinsic.
        DeclareLaunchArgument('fov_h_min_deg',      default_value='-13.5'),
        DeclareLaunchArgument('fov_h_max_deg',      default_value='17.0'),
        DeclareLaunchArgument('fov_v_min_deg',      default_value='-16.0'),
        DeclareLaunchArgument('fov_v_max_deg',      default_value='10.0'),

        GroupAction(actions=[
            PushRosNamespace('Multiespectral'),

            # LIDAR crop nodes — skipped in calibration mode (the full uncropped
            # cloud/images are stored directly by the compositor's _sync handlers).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share, '/launch/multiespectral_lidar_crop.launch.py']),
                condition=UnlessCondition(LaunchConfiguration('calibration_mode')),
                launch_arguments={
                    'fov_enabled':    LaunchConfiguration('fov_enabled'),
                    'fov_use_angular': LaunchConfiguration('fov_use_angular'),
                    'fov_h_min_deg':  LaunchConfiguration('fov_h_min_deg'),
                    'fov_h_max_deg':  LaunchConfiguration('fov_h_max_deg'),
                    'fov_v_min_deg':  LaunchConfiguration('fov_v_min_deg'),
                    'fov_v_max_deg':  LaunchConfiguration('fov_v_max_deg'),
                }.items(),
            ),

            # Buffer handlers (compositor)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_share, '/launch/multiespectral_buffer_handlers.launch.py']),
                launch_arguments={
                    'output_path':       output_path,
                    'main_trigger_topic': LaunchConfiguration('main_trigger_topic'),
                    'ouster_exposure_ns': LaunchConfiguration('ouster_exposure_ns'),
                    'gnss_topic':        LaunchConfiguration('gnss_topic'),
                    'odom_topic':        LaunchConfiguration('odom_topic'),
                    'main_trigger_hz':   frame_rate,
                    'calibration_mode':  LaunchConfiguration('calibration_mode'),
                    # Rate-cap the C++ ouster_sync_node's calib intermediate PUBLISH to
                    # the lidar bundle's on-disk budget (2 Hz in calib, matching the
                    # compositor's _lidar_store_hz; 0.0 = uncapped/inert in normal). The
                    # Python _store handlers store_all whatever the C++ publishes, so the
                    # cap MUST live here or calib floods the USB2 disk (audit gap G2).
                    'store_max_hz': PythonExpression([
                        "'2.0' if '", LaunchConfiguration('calibration_mode'),
                        "'.lower() in ('true', '1') else '0.0'",
                    ]),
                }.items(),
            ),
        ]),
    ])
