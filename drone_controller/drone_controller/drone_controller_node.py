#!/usr/bin/env python3
"""
Enhanced Drone Controller with better safety checks
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray
from sensor_msgs.msg import Imu, LaserScan
import numpy as np

class DroneFlappyController(Node):

    def __init__(self):
        super().__init__('drone_flappy_controller')
        
        # Declare parameters
        try:
            self.declare_parameter('forward_velocity', 1.5)
            self.declare_parameter('max_vertical_velocity', 2.5)
            self.declare_parameter('control_rate', 20.0)
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
        
        # Subscribers
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
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )
        
        self.ga_weights_sub = self.create_subscription(
            Float64MultiArray,
            '/ga/weights',
            self.ga_weights_callback,
            10
        )
        
        self.reset_sub = self.create_subscription(
            Bool,
            '/ga/reset_trigger',
            self.reset_callback,
            10
        )
        
        # State variables
        self.lidar_up_distance = None
        self.lidar_down_distance = None
        self.current_height = 0.0
        self.current_velocity_z = 0.0
        
        # GA weights
        self.ga_weights = None
        self.weights_received = False
        
        # Control state
        self.control_enabled = False
        self.initialization_phase = True
        self.init_start_time = None
        
        # Timer untuk control loop
        self.control_timer = self.create_timer(
            1.0 / control_rate,
            self.control_loop
        )
        
        self.get_logger().info('Enhanced Drone Controller initialized')

    def reset_callback(self, msg):
        """Handle reset from training loop"""
        if msg.data:
            self.get_logger().info('Reset received - entering initialization phase')
            self.initialization_phase = True
            self.init_start_time = self.get_clock().now()
            self.control_enabled = False
            
            # Send zero velocity during reset
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)

    def ga_weights_callback(self, msg):
        """Receive GA weights and enable control"""
        if len(msg.data) >= 4:
            self.ga_weights = list(msg.data[:4])
            
            if not self.weights_received:
                self.get_logger().info(f'GA Weights received: {[f"{w:.2f}" for w in self.ga_weights]}')
            
            self.weights_received = True
            
            # Enable control after short delay (Logic check here acts as immediate check)
            if self.init_start_time is not None:
                elapsed = (self.get_clock().now() - self.init_start_time).nanoseconds / 1e9
                if elapsed > 0.5:  # Wait 0.5s after reset
                    self.initialization_phase = False
                    self.control_enabled = True

    def lidar_up_callback(self, msg):
        """Process upward LiDAR with safety checks"""
        try:
            valid_ranges = [
                r for r in msg.ranges 
                if r < msg.range_max and r > msg.range_min and not np.isinf(r) and not np.isnan(r)
            ]
            
            if len(valid_ranges) > 0:
                self.lidar_up_distance = min(valid_ranges)
            else:
                self.lidar_up_distance = 10.0
                
        except Exception as e:
            self.get_logger().error(f'LiDAR up error: {e}')
            self.lidar_up_distance = 10.0

    def lidar_down_callback(self, msg):
        """Process downward LiDAR with safety checks"""
        try:
            valid_ranges = [
                r for r in msg.ranges 
                if r < msg.range_max and r > msg.range_min and not np.isinf(r) and not np.isnan(r)
            ]
            
            if len(valid_ranges) > 0:
                self.lidar_down_distance = min(valid_ranges)
            else:
                self.lidar_down_distance = 10.0
                
        except Exception as e:
            self.get_logger().error(f'LiDAR down error: {e}')
            self.lidar_down_distance = 10.0

    def odom_callback(self, msg):
        """Process odometry"""
        self.current_height = msg.pose.pose.position.z
        self.current_velocity_z = msg.twist.twist.linear.z

    def calculate_control_ga(self):
        """
        Calculate vertical control using GA weights
        Formula: vertical_cmd = w1*lidar_up + w2*lidar_down + w3*altitude + w4
        """
        
        # Safety checks
        if not self.weights_received or self.ga_weights is None:
            return 0.0
        
        if self.lidar_up_distance is None or self.lidar_down_distance is None:
            return 0.0
        
        # Extract weights
        w1, w2, w3, w4 = self.ga_weights
        
        # Normalize inputs to reasonable ranges
        lidar_up = np.clip(self.lidar_up_distance, 0.1, 10.0)
        lidar_down = np.clip(self.lidar_down_distance, 0.1, 10.0)
        altitude = np.clip(self.current_height, 0.0, 12.0)
        
        # Calculate command
        vertical_cmd = w1 * lidar_up + w2 * lidar_down + w3 * altitude + w4
        
        # Clamp output
        vertical_cmd = np.clip(
            vertical_cmd,
            -self.max_vertical_vel,
            self.max_vertical_vel
        )
        
        return vertical_cmd

    def control_loop(self):
        """Main control loop"""
        
        # FIX: Check timer BEFORE returning due to disabled control
        if not self.control_enabled and self.init_start_time is not None:
            elapsed = (self.get_clock().now() - self.init_start_time).nanoseconds / 1e9
            if elapsed > 0.5:
                self.initialization_phase = False
                self.control_enabled = True
                self.get_logger().info('Control enabled')
        
        # During initialization, send zero velocity
        if self.initialization_phase or not self.control_enabled:
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            return
        
        # Normal control
        if self.control_enabled:
            cmd_vel = Twist()
            cmd_vel.linear.x = self.forward_vel
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = self.calculate_control_ga()
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    
    controller = DroneFlappyController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()