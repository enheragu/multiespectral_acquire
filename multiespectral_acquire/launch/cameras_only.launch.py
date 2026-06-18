"""Camera acquisition only (Basler visible + FLIR LWIR).

Split out of multiespectral_launch.py so hitos_cameras.service can run the
cameras independently of the buffer compositor / crop nodes, which live in
capture_sync.launch.py (hitos_sync.service). This lets the sync process be
restarted without touching the cameras and staggers CPU/RAM startup peaks.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterValue

# Big-segment Fast DDS profile, applied per-process to the large-data publishers
# (here: visible/lwir cameras) so their 5.76 MB / 0.3 MB frames go over shared
# memory losslessly instead of falling back to fragmented UDP loopback. Per
# process only (segment is resident RAM); see the profile header.
_SHM_PROFILE = os.path.join(
    get_package_share_directory('hitos_setup'), 'config', 'fastdds_shm_profile.xml')


def generate_launch_description():
    session_folder = LaunchConfiguration('session_folder')
    frame_rate     = LaunchConfiguration('output_frame_rate')

    return LaunchDescription([
        # ---- Arguments ----
        DeclareLaunchArgument('session_folder',     default_value='test_session'),
        DeclareLaunchArgument('output_frame_rate',  default_value='1'),

        # Camera PTP. With lwir_use_ptp=true the FLIR adapter verifies an
        # ACTUAL PTP lock (ptpServoStatus==Locked + sane probe-frame timestamp)
        # and falls back to software TimestampCalibration otherwise. Note: the
        # A68 only steps its clock on a cold lock — it usually needs to boot
        # with the PTP grandmaster already running.
        DeclareLaunchArgument('visible_use_ptp',    default_value='false'),
        DeclareLaunchArgument('lwir_use_ptp',       default_value='true'),

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
                additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': _SHM_PROFILE},
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
                additional_env={'FASTRTPS_DEFAULT_PROFILES_FILE': _SHM_PROFILE},
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
        ]),
    ])
