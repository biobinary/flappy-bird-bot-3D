from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    """Main launch file untuk simulasi drone dengan GA"""
    
    # Get package paths
    drone_world_pkg = FindPackageShare('drone_world')
    drone_controller_pkg = FindPackageShare('drone_controller')
    drone_description_pkg = FindPackageShare('drone_description') # Tambahan
    
    # World file path
    world_file = PathJoinSubstitution([
        drone_world_pkg,
        'worlds',
        'drone_arena.sdf'
    ])

    # Model path (lokasi folder models di dalam install/share)
    model_path = PathJoinSubstitution([
        drone_description_pkg,
        'models'
    ])
    
    return LaunchDescription([
        # --- FIX: Tambahkan path model ke environment variable Ignition Gazebo ---
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=model_path
        ),
        # ------------------------------------------------------------------------

        # Launch Arguments
        DeclareLaunchArgument(
            'world',
            default_value=[world_file],
            description='Path to world file'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=[
                'ign', 'gazebo',
                LaunchConfiguration('world'),
                '-r', # Auto start simulation
                '--verbose'
            ],
            output='screen'
        ),
        
        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/model/quadrotor/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
                '/model/quadrotor/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/model/quadrotor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            ],
            output='screen'
        ),
        
        # Drone Controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    drone_controller_pkg,
                    'launch',
                    'controller.launch.py'
                ])
            ])
        ),
        
        # Collision Monitor
        Node(
            package='drone_collision',
            executable='collision_monitor.py',
            name='collision_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),
        
        # Fitness Evaluator
        Node(
            package='drone_fitness',
            executable='fitness_evaluator.py',
            name='fitness_evaluator',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),
    ])