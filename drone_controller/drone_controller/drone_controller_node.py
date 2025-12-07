#!/usr/bin/env python3

"""
Drone Controller Node for Flappy Bird Simulation with GA Integration
Reads sensor data, GA weights, and publishes control commands
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import Imu, LaserScan
import numpy as np

class DroneFlappyController(Node):

    """
    Controller node untuk drone Flappy Bird
    Menggunakan bobot dari GA untuk kontrol
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
        
        # Subscribers - Sensors
        self.lidar_up_sub = self.create_subscription(
            LaserScan,              
            '/drone/lidar/up',
            self.lidar_up_callback,
            10 
        )
        
        self.lidar_down_sub = self.create_subscription(
            LaserScan,
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
            '/drone/odom',  # Updated to match remapping in launch file
            self.odom_callback,
            10
        )
        
        # Subscriber - GA Weights
        self.ga_weights_sub = self.create_subscription(
            Float64MultiArray,
            '/ga/weights',
            self.ga_weights_callback,
            10
        )
        
        # State variables - Sensors
        self.lidar_up_distance = None
        self.lidar_down_distance = None
        self.current_imu = None
        self.current_odom = None
        self.current_height = 0.0
        self.current_velocity_z = 0.0
        
        # State variables - GA Weights
        self.ga_weights = None  # [w1, w2, w3, w4]
        self.weights_received = False
        
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
        
        self.get_logger().info('Drone Flappy Controller Node initialized with GA Integration')
        self.get_logger().info(f'Forward velocity: {self.forward_vel} m/s')
        self.get_logger().info(f'Max vertical velocity: {self.max_vertical_vel} m/s')
        self.get_logger().info(f'Control rate: {control_rate} Hz')
        self.get_logger().info('Waiting for GA weights from /ga/weights...')

    def ga_weights_callback(self, msg):
        """
        Callback untuk menerima bobot GA dari topik /ga/weights
        Format: [w1, w2, w3, w4]
        """
        if len(msg.data) >= 4:
            self.ga_weights = list(msg.data[:4])
            
            if not self.weights_received:
                self.get_logger().info(f'GA Weights received: {self.ga_weights}')
                self.weights_received = True
            else:
                self.get_logger().debug(f'GA Weights updated: {self.ga_weights}')
        else:
            self.get_logger().warn(f'Invalid GA weights received. Expected 4 values, got {len(msg.data)}')

    def lidar_up_callback(self, msg):
        """Process upward LiDAR Scan"""
        try:
            valid_ranges = [r for r in msg.ranges if r < msg.range_max and r > msg.range_min]
            
            if len(valid_ranges) > 0:
                self.lidar_up_distance = min(valid_ranges)
            else:
                self.lidar_up_distance = msg.range_max  # Use max range if no valid readings
                
        except Exception as e:
            self.get_logger().error(f'Error processing lidar_up: {e}')
            self.lidar_up_distance = 10.0

    def lidar_down_callback(self, msg):
        """Process downward LiDAR Scan"""
        try:
            valid_ranges = [r for r in msg.ranges if r < msg.range_max and r > msg.range_min]
            
            if len(valid_ranges) > 0:
                self.lidar_down_distance = min(valid_ranges)
            else:
                self.lidar_down_distance = msg.range_max
                
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

    def calculate_control_ga(self):
        """
        Calculate vertical control command using GA weights
        
        Formula: vertical_cmd = w1*lidar_up + w2*lidar_down + w3*altitude + w4
        
        Returns:
            float: vertical velocity command (-max_vertical_vel to +max_vertical_vel)
        """
        
        # Safety check - if weights not received, return 0
        if not self.weights_received or self.ga_weights is None:
            return 0.0
        
        # Safety check - if sensor data not available
        if self.lidar_up_distance is None or self.lidar_down_distance is None:
            self.get_logger().warn('LiDAR data not available yet')
            return 0.0
        
        # Extract weights
        w1, w2, w3, w4 = self.ga_weights
        
        # Get sensor inputs
        lidar_up = self.lidar_up_distance
        lidar_down = self.lidar_down_distance
        altitude = self.current_height
        
        # Calculate vertical command using GA weights
        # vertical_cmd = w1*lidar_up + w2*lidar_down + w3*altitude + w4
        vertical_cmd = w1 * lidar_up + w2 * lidar_down + w3 * altitude + w4
        
        # Clamp to max velocity
        vertical_cmd = np.clip(
            vertical_cmd,
            -self.max_vertical_vel,
            self.max_vertical_vel
        )
        
        # Log debug info periodically (every ~1 second)
        if self.get_clock().now().nanoseconds % 1000000000 < 50000000:
            self.get_logger().info(
                f'Sensors - Up: {lidar_up:.2f}m, Down: {lidar_down:.2f}m, '
                f'Alt: {altitude:.2f}m | Cmd_Z: {vertical_cmd:.2f}m/s'
            )
        
        return vertical_cmd

    def control_loop(self):
        """Main control loop - publishes cmd_vel using GA weights"""
        
        if not self.control_enabled:
            return
        
        # Create Twist message
        cmd_vel = Twist()
        
        # Constant forward velocity
        cmd_vel.linear.x = self.forward_vel
        cmd_vel.linear.y = 0.0
        
        # Calculate vertical velocity using GA weights
        cmd_vel.linear.z = self.calculate_control_ga()
        
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
            self.weights_received and
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
        Get current sensor state for monitoring
        
        Returns:
            dict: Dictionary containing sensor data
        """
        return {
            'lidar_up': self.lidar_up_distance,
            'lidar_down': self.lidar_down_distance,
            'height': self.current_height,
            'velocity_z': self.current_velocity_z,
            'ga_weights': self.ga_weights,
            'weights_received': self.weights_received
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