from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value=EnvironmentVariable('DHT22_PORT', default_value=''),
            description='Serial port for the ESP8266 (empty = auto-detect)',
        ),
        DeclareLaunchArgument('baudrate', default_value='74880'),
        DeclareLaunchArgument('frame_id', default_value='dht22_link'),

        Node(
            package='temperature_driver',
            executable='dht22_node.py',
            name='dht22',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_id': LaunchConfiguration('frame_id'),
                'retry_interval': 5.0,
            }],
        ),
    ])
