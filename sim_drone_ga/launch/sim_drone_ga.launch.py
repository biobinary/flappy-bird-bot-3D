from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # Get package paths
    drone_world_pkg = FindPackageShare('drone_world')
    drone_controller_pkg = FindPackageShare('drone_controller')
    drone_description_pkg = FindPackageShare('drone_description')
    
    # World file path - Updated to use flappy.sdf
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
        
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[models_path]
        ),

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
        
        # Ignition Gazebo
        ExecuteProcess(
            cmd=[
                'ign', 'gazebo', 
                LaunchConfiguration('world'),
                '-r',
                '-v', '4'
            ],
            output='screen'
        ),
        
        # ROS GZ Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            arguments=[
                '/model/quadrotor/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/drone/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                '/world/flappy/physics/contacts@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                '/model/quadrotor/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                '/drone/lidar/up@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/drone/lidar/down@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/world/flappy/control@ros_gz_interfaces/srv/ControlWorld]gz.msgs.WorldControl',
                '/world/flappy/control@ros_gz_interfaces/srv/ControlWorld@gz.msgs.WorldControl@gz.msgs.Boolean'
            ],
            output='screen',
            remappings=[
                ('/model/quadrotor/odometry', '/drone/odom')
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }]
        ),
        
        # Drone Controller
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    drone_controller_pkg,
                    'launch',
                    'controller.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        ),
        
        # Collision Monitor
        Node(
            package='drone_collision',
            executable='contact_monitor.py',
            name='contact_monitor',
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