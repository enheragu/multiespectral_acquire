from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        GroupAction([
            PushRosNamespace('Multiespectral'),
            Node(
                package='multiespectral_acquire_gui',
                executable='multiespectral_control',
                name='control_gui',
                output='screen'
            )
        ])
    ])
