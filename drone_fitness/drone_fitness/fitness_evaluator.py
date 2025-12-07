#!/usr/bin/env python3
"""
Drone Fitness Evaluator Node
Menghitung fitness berdasarkan jarak maksimum sumbu X yang dicapai drone sebelum collision.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64


class DroneFitness(Node):
    """Node untuk evaluasi fitness Genetic Algorithm."""

    def __init__(self):
        super().__init__('drone_fitness')
        
        # Publisher untuk fitness episode
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
        
        # State variables
        self.initial_x = None
        self.max_x = 0.0
        self.current_x = 0.0
        self.collision_detected = False
        
        # Timer untuk publish current max fitness secara periodik (monitoring)
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info('Drone Fitness Evaluator Node started')
        self.get_logger().info('Waiting for initial odometry to set starting position')

    def odom_callback(self, msg):
        """Callback untuk odometry - update posisi X dan max X."""
        self.current_x = msg.pose.pose.position.x
        
        # Set initial_x pada pesan odom pertama
        if self.initial_x is None:
            self.initial_x = self.current_x
            self.max_x = self.current_x
            self.get_logger().info(f'Initial X position set: {self.initial_x:.2f}')
        
        # Update max X
        if self.current_x > self.max_x:
            self.max_x = self.current_x

    def collision_callback(self, msg):
        """Callback untuk collision - hitung dan publish fitness final, lalu reset."""
        if msg.data and not self.collision_detected:
            # Collision terdeteksi untuk pertama kali - finalkan fitness
            travelled_distance = self.max_x - self.initial_x
            fitness_msg = Float64()
            fitness_msg.data = float(travelled_distance)
            self.fitness_pub.publish(fitness_msg)
            
            self.get_logger().warn(f'COLLISION DETECTED! Final Fitness (Max Travelled X): {travelled_distance:.2f}')
            
            # Reset untuk episode berikutnya
            self.max_x = self.initial_x if self.initial_x is not None else 0.0
            self.collision_detected = False
            self.get_logger().info('Fitness episode reset - ready for next run')

    def timer_callback(self):
        """Timer callback - publish current max travelled distance untuk monitoring."""
        if self.initial_x is not None:
            current_max_travelled = self.max_x - self.initial_x
            fitness_msg = Float64()
            fitness_msg.data = float(current_max_travelled)
            self.fitness_pub.publish(fitness_msg)
            
            self.get_logger().debug(f'Current Max Travelled X: {current_max_travelled:.2f}')


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