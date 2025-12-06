from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file untuk Genetic Algorithm"""
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'population_size',
            default_value='50',
            description='Population size for GA'
        ),
        
        DeclareLaunchArgument(
            'max_generations',
            default_value='100',
            description='Maximum number of generations'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        Node(
            package='drone_ga',
            executable='ga_node.py',
            name='genetic_algorithm',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'population_size': LaunchConfiguration('population_size'),
                'max_generations': LaunchConfiguration('max_generations'),
            }]
        ),
    ])

