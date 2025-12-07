#!/usr/bin/env python3
"""
Fixed Drone Fitness Evaluator Node
Properly handles episode-based fitness evaluation
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


class DroneFitness(Node):
    """Node untuk evaluasi fitness Genetic Algorithm."""

    def __init__(self):
        super().__init__('drone_fitness')
        
        # Publisher untuk fitness
        self.fitness_pub = self.create_publisher(
            Float64,
            '/ga/fitness',
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            10
        )
        
        self.collision_sub = self.create_subscription(
            Bool,
            '/drone/collision',
            self.collision_callback,
            10
        )
        
        # Listen to reset trigger from training loop
        self.reset_sub = self.create_subscription(
            Bool,
            '/ga/reset_trigger',
            self.reset_callback,
            10
        )
        
        # State variables
        self.initial_x = None
        self.max_x = 0.0
        self.current_x = 0.0
        self.episode_active = False
        self.collision_handled = False
        
        # Timer untuk publish continuous fitness (monitoring only)
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        self.get_logger().info('Drone Fitness Evaluator Node started (Fixed)')

    def reset_callback(self, msg):
        """Handle reset signal from training loop"""
        if msg.data:
            self.get_logger().info('Reset signal received - starting new episode')
            self.initial_x = None
            self.max_x = 0.0
            self.current_x = 0.0
            self.episode_active = True
            self.collision_handled = False

    def odom_callback(self, msg):
        """Callback untuk odometry - update posisi X dan max X."""
        self.current_x = msg.pose.pose.position.x
        
        # Set initial_x pada pesan odom pertama setelah reset
        if self.initial_x is None and self.episode_active:
            self.initial_x = self.current_x
            self.max_x = self.current_x
            self.get_logger().info(f'Episode started at X = {self.initial_x:.2f}')
        
        # Update max X hanya jika episode aktif
        if self.episode_active and self.initial_x is not None:
            if self.current_x > self.max_x:
                self.max_x = self.current_x

    def collision_callback(self, msg):
        """
        Callback untuk collision
        Hanya handle collision pertama per episode
        """
        if msg.data and self.episode_active and not self.collision_handled:
            # Mark collision as handled untuk episode ini
            self.collision_handled = True
            self.episode_active = False
            
            # Calculate final fitness
            if self.initial_x is not None:
                travelled_distance = self.max_x - self.initial_x
            else:
                travelled_distance = 0.0
            
            # Publish final fitness
            fitness_msg = Float64()
            fitness_msg.data = max(0.0, float(travelled_distance))
            self.fitness_pub.publish(fitness_msg)
            
            self.get_logger().info(
                f'Episode ended - Travelled: {travelled_distance:.2f}m '
                f'(from {self.initial_x:.2f} to {self.max_x:.2f})'
            )

    def timer_callback(self):
        """
        Timer callback - publish current fitness untuk monitoring
        Ini TIDAK akan digunakan untuk evolusi, hanya untuk visualisasi
        """
        if self.episode_active and self.initial_x is not None:
            current_distance = self.max_x - self.initial_x
            
            # Publish untuk monitoring
            fitness_msg = Float64()
            fitness_msg.data = max(0.0, float(current_distance))
            self.fitness_pub.publish(fitness_msg)


def main(args=None):
    rclpy.init(args=args)
    
    evaluator = DroneFitness()
    
    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        pass
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()