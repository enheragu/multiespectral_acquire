import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Big-segment Fast DDS profile (shared memory) for the compositor process: it
# republishes the large _sync clouds/images (5.76 MB visible, 3.8 MB cloud), so
# SHM keeps them off fragmented UDP loopback. Per-process (resident-RAM segment);
# see the profile header in hitos_setup/config/.
_SHM_PROFILE = os.path.join(
    get_package_share_directory('hitos_setup'), 'config', 'fastdds_shm_profile.xml')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_path'),
        DeclareLaunchArgument('main_trigger_topic'),
        DeclareLaunchArgument('ouster_exposure_ns', default_value='30000'),
        DeclareLaunchArgument('gnss_topic',  default_value='/gnss/fix'),
        DeclareLaunchArgument('odom_topic',  default_value='/odometry/combined'),
        DeclareLaunchArgument('main_trigger_hz', default_value='0.0'),
        DeclareLaunchArgument('calibration_mode', default_value='false'),
        DeclareLaunchArgument('disable_ouster_sync', default_value='true'),
        # Calib on-disk store-rate cap for the lidar bundle (cloud + 4 images).
        # Fed to the C++ ouster_sync_node so it rate-caps which intermediates it
        # PUBLISHES on *_sync (the Python _store handlers store_all whatever lands).
        # Higher-level calib launch sets store_max_hz:=2.0 alongside
        # calibration_mode:=true; normal mode leaves it 0.0 (uncapped/inert). This
        # is a real float arg (NOT a raw-string compare on calibration_mode), so it
        # matches the compositor's _lidar_store_hz=2.0-if-calib value regardless of
        # how calibration_mode is spelled ('true'/'True'/'1'/...).
        DeclareLaunchArgument('store_max_hz', default_value='0.0'),

        Node(
            package='multiespectral_acquire',
            executable='buffer_compositor_node.py',
            name='buffer_compositor',
            output='screen',
            additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': _SHM_PROFILE},
            parameters=[{
                'output_path':          LaunchConfiguration('output_path'),
                'main_trigger_topic':   LaunchConfiguration('main_trigger_topic'),
                'ouster_exposure_ns':   LaunchConfiguration('ouster_exposure_ns'),
                'gnss_topic':           LaunchConfiguration('gnss_topic'),
                'odom_topic':           LaunchConfiguration('odom_topic'),
                'main_trigger_hz':      ParameterValue(LaunchConfiguration('main_trigger_hz'), value_type=float),
                'calibration_mode':     ParameterValue(LaunchConfiguration('calibration_mode'), value_type=bool),
                'disable_ouster_sync':  ParameterValue(LaunchConfiguration('disable_ouster_sync'), value_type=bool),
            }],
        ),

        # C++ Ouster sync node — replaces the 5 GIL-bound Python _sync handlers when
        # disable_ouster_sync is true. Publishes the identical ouster/*_sync topics
        # (no GIL → recovers image yield). Reversible: disable_ouster_sync:=false.
        Node(
            package='multiespectral_acquire',
            executable='ouster_sync_node',
            name='ouster_sync_node',
            output='screen',
            # Runs in BOTH normal and calib (calib is in-motion → needs the no-GIL yield
            # too). It only republishes ouster/*_sync; storage differs by mode: normal
            # crops then stores *_sync_cropped, calib stores *_sync (uncropped) directly.
            condition=IfCondition(LaunchConfiguration('disable_ouster_sync')),
            additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': _SHM_PROFILE},
            parameters=[{
                'main_trigger_topic':  LaunchConfiguration('main_trigger_topic'),
                'clock_offset_topic':  '/ouster/clock_offset',
                'output_suffix':       '_sync',
                # Calib intermediate-flush wiring. calibration_mode normalized through
                # ROS bool (same as the compositor at line 43); store_max_hz is a real
                # float derived from the SAME normalized switch upstream (=2.0 in calib,
                # 0.0 normal), so the C++ rate-cap always agrees with the compositor's
                # _lidar_store_hz. Defaults (false / 0.0) => calib logic dead => normal
                # mode byte-identical.
                'calibration_mode':    ParameterValue(LaunchConfiguration('calibration_mode'), value_type=bool),
                'store_max_hz':        ParameterValue(LaunchConfiguration('store_max_hz'), value_type=float),
            }],
        ),
    ])
