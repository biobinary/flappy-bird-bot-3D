from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    AppendEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Complete GA Training Launch File
    Launches entire simulation stack + GA training loop
    """
    
    # Get package paths
    drone_world_pkg = FindPackageShare('drone_world')
    drone_controller_pkg = FindPackageShare('drone_controller')
    drone_description_pkg = FindPackageShare('drone_description')
    
    # World file path
    world_file = PathJoinSubstitution([
        drone_world_pkg,
        'worlds',
        'flappy.sdf'
    ])

    models_path = PathJoinSubstitution([
        drone_description_pkg,
        'models'
    ])

    return LaunchDescription([
        
        # Environment variable for model loading
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[models_path]
        ),

        # ==================== LAUNCH ARGUMENTS ====================
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
            default_value='30',
            description='GA population size'
        ),
        
        DeclareLaunchArgument(
            'max_generations',
            default_value='100',
            description='Maximum number of generations'
        ),
        
        DeclareLaunchArgument(
            'episode_timeout',
            default_value='15.0',
            description='Episode timeout in seconds'
        ),
        
        DeclareLaunchArgument(
            'forward_velocity',
            default_value='1.5',
            description='Drone forward velocity (m/s)'
        ),
        
        DeclareLaunchArgument(
            'max_vertical_velocity',
            default_value='2.5',
            description='Maximum vertical velocity (m/s)'
        ),
        
        # ==================== GAZEBO IGNITION ====================
        ExecuteProcess(
            cmd=[
                'ign', 'gazebo', 
                LaunchConfiguration('world'),
                '-r',  # Start paused
                '-v', '3'  # Verbosity level
            ],
            output='screen',
            shell=False
        ),
        
        # ==================== ROS-GAZEBO BRIDGE ====================
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='ros_gz_bridge',
                    executable='parameter_bridge',
                    name='ros_gz_bridge',
                    arguments=[
                        # Topic Mappings (Existing)
                        '/model/quadrotor/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                        '/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/world/flappy/physics/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                        '/world/flappy/model/quadrotor/link/base_link/sensor/collision_sensor/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                        '/model/quadrotor/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/drone/lidar/up@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/drone/lidar/down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/world/flappy/control@ros_gz_interfaces/srv/ControlWorld@ignition.msgs.WorldControl@ignition.msgs.Boolean'
                    ],
                    output='screen',
                    remappings=[
                        ('/model/quadrotor/odometry', '/drone/odom'),
                        ('/world/flappy/model/quadrotor/link/base_link/sensor/collision_sensor/contact', '/drone/contacts')
                    ],
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }]
                ),
            ]
        ),
        
        # ==================== DRONE CONTROLLER ====================
        TimerAction(
            period=5.0,  # Wait for bridge
            actions=[
                Node(
                    package='drone_controller',
                    executable='drone_controller_node.py',
                    name='drone_flappy_controller',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'forward_velocity': LaunchConfiguration('forward_velocity'),
                        'max_vertical_velocity': LaunchConfiguration('max_vertical_velocity'),
                        'control_rate': 20.0,
                    }]
                ),
            ]
        ),
        
        # ==================== COLLISION MONITOR ====================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='drone_collision',
                    executable='contact_monitor.py',
                    name='contact_monitor',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }]
                ),
            ]
        ),
        
        # ==================== FITNESS EVALUATOR ====================
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='drone_fitness',
                    executable='fitness_evaluator.py',
                    name='fitness_evaluator',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                    }]
                ),
            ]
        ),
        
        # ==================== GA TRAINING LOOP ====================
        TimerAction(
            period=8.0,  # Wait for all other nodes to be ready
            actions=[
                Node(
                    package='drone_ga',
                    executable='ga_node.py',
                    name='ga_training_loop',
                    output='screen',
                    parameters=[{
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'population_size': LaunchConfiguration('population_size'),
                        'max_generations': LaunchConfiguration('max_generations'),
                        'episode_timeout': LaunchConfiguration('episode_timeout'),
                        'settling_time': 1.0,
                    }]
                ),
            ]
        ),
    ])