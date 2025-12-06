#!/usr/bin/env python3
"""
Drone Controller Node for Flappy Bird Simulation
Reads sensor data and publishes control commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np


class DroneFlappyController(Node):
    """
    Controller node untuk drone Flappy Bird
    Membaca sensor dan menghasilkan kontrol
    """

    def __init__(self):
        super().__init__('drone_flappy_controller')
        
        # Declare parameters (skip use_sim_time as it's auto-declared)
        try:
            self.declare_parameter('forward_velocity', 1.0)
            self.declare_parameter('max_vertical_velocity', 2.0)
            self.declare_parameter('control_rate', 20.0)  # Hz
        except Exception as e:
            self.get_logger().warn(f'Parameter declaration warning: {e}')
        
        # Get parameters
        self.forward_vel = self.get_parameter('forward_velocity').value
        self.max_vertical_vel = self.get_parameter('max_vertical_velocity').value
        control_rate = self.get_parameter('control_rate').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/model/quadrotor/cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            Bool,
            '/drone/controller/status',
            10
        )
        
        # Subscribers
        self.lidar_up_sub = self.create_subscription(
            PointCloud2,
            '/drone/lidar/up',
            self.lidar_up_callback,
            10
        )
        
        self.lidar_down_sub = self.create_subscription(
            PointCloud2,
            '/drone/lidar/down',
            self.lidar_down_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/drone/imu',
            self.imu_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odometry',
            self.odom_callback,
            10
        )
        
        # State variables
        self.lidar_up_distance = None
        self.lidar_down_distance = None
        self.current_imu = None
        self.current_odom = None
        self.current_height = 0.0
        self.current_velocity_z = 0.0
        
        # Control state
        self.is_active = True
        self.control_enabled = True
        
        # Timer untuk control loop
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )
        
        # Timer untuk status publishing
        self.status_timer = self.create_timer(
            1.0,
            self.publish_status
        )
        
        self.get_logger().info('Drone Flappy Controller Node initialized')
        self.get_logger().info(f'Forward velocity: {self.forward_vel} m/s')
        self.get_logger().info(f'Max vertical velocity: {self.max_vertical_vel} m/s')
        self.get_logger().info(f'Control rate: {control_rate} Hz')

    def lidar_up_callback(self, msg):
        """Process upward LiDAR point cloud"""
        try:
            # Extract points from point cloud
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(point)
            
            if len(points) > 0:
                # Calculate minimum distance from forward-facing points
                distances = []
                for x, y, z in points:
                    # Filter points in front of drone (positive x direction)
                    if x > 0:
                        distance = np.sqrt(x**2 + y**2 + z**2)
                        distances.append(distance)
                
                if len(distances) > 0:
                    self.lidar_up_distance = min(distances)
                else:
                    self.lidar_up_distance = 10.0  # Max range
            else:
                self.lidar_up_distance = 10.0
                
        except Exception as e:
            self.get_logger().error(f'Error processing lidar_up: {e}')
            self.lidar_up_distance = 10.0

    def lidar_down_callback(self, msg):
        """Process downward LiDAR point cloud"""
        try:
            # Extract points from point cloud
            points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append(point)
            
            if len(points) > 0:
                # Calculate minimum distance from forward-facing points
                distances = []
                for x, y, z in points:
                    # Filter points in front of drone (positive x direction)
                    if x > 0:
                        distance = np.sqrt(x**2 + y**2 + z**2)
                        distances.append(distance)
                
                if len(distances) > 0:
                    self.lidar_down_distance = min(distances)
                else:
                    self.lidar_down_distance = 10.0  # Max range
            else:
                self.lidar_down_distance = 10.0
                
        except Exception as e:
            self.get_logger().error(f'Error processing lidar_down: {e}')
            self.lidar_down_distance = 10.0

    def imu_callback(self, msg):
        """Process IMU data"""
        self.current_imu = msg

    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_odom = msg
        self.current_height = msg.pose.pose.position.z
        self.current_velocity_z = msg.twist.twist.linear.z

    def calculate_control(self):
        """
        Calculate vertical control command based on sensor data
        
        PLACEHOLDER CONTROLLER - Replace this with GA-based controller
        
        Returns:
            float: vertical velocity command (-max_vertical_vel to +max_vertical_vel)
        """
        
        # Safety check
        if self.lidar_up_distance is None or self.lidar_down_distance is None:
            self.get_logger().warn('LiDAR data not available yet')
            return 0.0
        
        # Simple rule-based controller (placeholder for GA)
        # This will be replaced by neural network or GA-evolved controller
        
        # Calculate clearance
        up_clearance = self.lidar_up_distance
        down_clearance = self.lidar_down_distance
        
        # Simple proportional controller
        # Try to maintain equal distance from ceiling and floor
        target_clearance = 3.0  # meters
        
        vertical_command = 0.0
        
        # If too close to ceiling, descend
        if up_clearance < target_clearance:
            vertical_command = -(target_clearance - up_clearance) * 0.5
        
        # If too close to floor, ascend
        if down_clearance < target_clearance:
            vertical_command = (target_clearance - down_clearance) * 0.5
        
        # Clamp to max velocity
        vertical_command = np.clip(
            vertical_command,
            -self.max_vertical_vel,
            self.max_vertical_vel
        )
        
        # Log debug info
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # Log every ~1 second
            self.get_logger().info(
                f'Clearance - Up: {up_clearance:.2f}m, Down: {down_clearance:.2f}m, '
                f'Height: {self.current_height:.2f}m, Cmd_Z: {vertical_command:.2f}m/s'
            )
        
        return vertical_command

    def control_loop(self):
        """Main control loop - publishes cmd_vel"""
        
        if not self.control_enabled:
            return
        
        # Create Twist message
        cmd_vel = Twist()
        
        # Constant forward velocity
        cmd_vel.linear.x = self.forward_vel
        cmd_vel.linear.y = 0.0
        
        # Calculate vertical velocity
        cmd_vel.linear.z = self.calculate_control()
        
        # No angular velocity for Flappy Bird
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)

    def publish_status(self):
        """Publish controller status"""
        status = Bool()
        status.data = (
            self.is_active and 
            self.lidar_up_distance is not None and 
            self.lidar_down_distance is not None
        )
        self.status_pub.publish(status)

    def enable_control(self):
        """Enable control output"""
        self.control_enabled = True
        self.get_logger().info('Control enabled')

    def disable_control(self):
        """Disable control output"""
        self.control_enabled = False
        # Send zero velocity
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().info('Control disabled')

    def get_sensor_state(self):
        """
        Get current sensor state for GA/ML processing
        
        Returns:
            dict: Dictionary containing sensor data
        """
        return {
            'lidar_up': self.lidar_up_distance,
            'lidar_down': self.lidar_down_distance,
            'height': self.current_height,
            'velocity_z': self.current_velocity_z,
            'imu': self.current_imu
        }


def main(args=None):
    rclpy.init(args=args)
    
    controller = DroneFlappyController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down controller...')
    finally:
        controller.disable_control()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()