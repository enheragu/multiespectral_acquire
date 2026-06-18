import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def _str(arg):
    """Wrap a LaunchConfiguration substitution as an explicit string parameter."""
    return ParameterValue(LaunchConfiguration(arg), value_type=str)


def generate_launch_description():
    # ament_python packages install executables to bin/, not lib/<pkg>/.
    # Resolve the path explicitly so ros2 launch doesn't require the libexec dir.
    executable = os.path.join(
        get_package_prefix('multiespectral_acquire_gui'), 'bin', 'multiespectral_control')

    return LaunchDescription([
        DeclareLaunchArgument('gui_title',     default_value='Camera Acquisition GUI'),
        DeclareLaunchArgument('flask_host',    default_value='0.0.0.0'),
        DeclareLaunchArgument('flask_port',    default_value='5000'),
        DeclareLaunchArgument('namespace',     default_value='camera'),
        DeclareLaunchArgument('camera1_topic', default_value='/camera1/image'),
        DeclareLaunchArgument('camera1_name',  default_value='Camera 1'),
        DeclareLaunchArgument('camera2_topic', default_value='/camera2/image'),
        DeclareLaunchArgument('camera2_name',  default_value='Camera 2'),
        DeclareLaunchArgument('lidar_topic',   default_value=''),
        DeclareLaunchArgument('lidar_name',    default_value='LIDAR'),

        GroupAction(actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            Node(
                executable=executable,
                name='multiespectral_flask_gui',
                output='screen',
                parameters=[{
                    'flask_host':    _str('flask_host'),
                    'flask_port':    LaunchConfiguration('flask_port'),
                    'gui_title':     _str('gui_title'),
                    'camera1_topic': _str('camera1_topic'),
                    'camera1_name':  _str('camera1_name'),
                    'camera2_topic': _str('camera2_topic'),
                    'camera2_name':  _str('camera2_name'),
                    'lidar_topic':   _str('lidar_topic'),
                    'lidar_name':    _str('lidar_name'),
                }],
            ),
        ]),
    ])
