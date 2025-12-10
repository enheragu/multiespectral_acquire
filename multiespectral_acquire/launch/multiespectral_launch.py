from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Argumentos

    dataset_output_path = DeclareLaunchArgument('dataset_output_path', 
                                                default_value='/home/quique/umh/ros2_ws/images_eeha')
    frame_rate = DeclareLaunchArgument('frame_rate', default_value='1')
    
    # Namespace Multiespectral
    multiespectral_ns = GroupAction([

        # FLIR LWIR (slave)
        Node(
            package='multiespectral_acquire',
            executable='flir_slave',
            name='lwir_camera',
            namespace='Multiespectral',
            output='screen',
            parameters=[{
                'dataset_output_path': LaunchConfiguration('dataset_output_path'),
                'frame_rate': LaunchConfiguration('frame_rate'),
                'image_topic': 'lwir_camera',
                'camera_info_url': ['file://', PathJoinSubstitution([
                    FindPackageShare('multiespectral_acquire'), 
                    'conf', 'lwir_params.yaml'])]
            }],
            emulate_tty=True # Emulate colors and such, if not colors and formatting are lost when using launch
        ),
        
        # Basler visible (master) - IP desde variable de entorno
        Node(
            package='multiespectral_acquire',
            executable='basler_master',
            name='visible_camera',
            namespace='Multiespectral',
            output='screen',
            parameters=[{
                'dataset_output_path': LaunchConfiguration('dataset_output_path'),
                'camera_ip': '$(env MULTIESPECTRAL_VISIBLE_IP)',
                'frame_rate': LaunchConfiguration('frame_rate'),
                'image_topic': 'visible_camera',
                'camera_info_url': ['file://', PathJoinSubstitution([
                    FindPackageShare('multiespectral_acquire'), 
                    'conf', 'visible_params.yaml'])]
            }],
            emulate_tty=True # Emulate colors and such, if not colors and formatting are lost when using launch
            # , arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])
    
    return LaunchDescription([
        dataset_output_path,
        frame_rate,
        multiespectral_ns
    ])
