from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Full system launch file dengan semua komponen termasuk GA"""
    
    # Get package paths
    drone_world_pkg = FindPackageShare('drone_world')
    drone_controller_pkg = FindPackageShare('drone_controller')
    drone_ga_pkg = FindPackageShare('drone_ga')
    
    # World file path
    world_file = PathJoinSubstitution([
        drone_world_pkg,
        'worlds',
        'drone_arena.sdf'
    ])
    
    return LaunchDescription([
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
        
        DeclareLaunchArgument(
            'population_size',
            default_value='50',
            description='GA population size'
        ),
        
        DeclareLaunchArgument(
            'max_generations',
            default_value='100',
            description='GA max generations'
        ),
        
        # Launch Ignition Gazebo
        ExecuteProcess(
            cmd=[
                'ign', 'gazebo',
                LaunchConfiguration('world'),
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
        
        # Genetic Algorithm
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    drone_ga_pkg,
                    'launch',
                    'ga.launch.py'
                ])
            ])
        ),
    ])

