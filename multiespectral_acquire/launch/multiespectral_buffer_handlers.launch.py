import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
            }],
        ),
    ])
