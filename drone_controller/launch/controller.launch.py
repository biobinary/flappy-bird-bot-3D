from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file untuk drone flappy controller"""
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'forward_velocity',
            default_value='1.0',
            description='Constant forward velocity (m/s)'
        ),
        
        DeclareLaunchArgument(
            'max_vertical_velocity',
            default_value='2.0',
            description='Maximum vertical velocity (m/s)'
        ),
        
        DeclareLaunchArgument(
            'control_rate',
            default_value='20.0',
            description='Control loop rate (Hz)'
        ),
        
        Node(
            package='drone_controller',
            executable='drone_controller_node.py',
            name='drone_flappy_controller',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'forward_velocity': LaunchConfiguration('forward_velocity'),
                'max_vertical_velocity': LaunchConfiguration('max_vertical_velocity'),
                'control_rate': LaunchConfiguration('control_rate'),
            }],
            remappings=[
                # Remap topics if needed
            ]
        ),
    ])