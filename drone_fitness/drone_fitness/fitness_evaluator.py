#!/usr/bin/env python3
"""
Fitness Evaluator Node
Menghitung fitness score berdasarkan performa drone
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
import math
import numpy as np


class FitnessEvaluator(Node):
    """Node untuk evaluasi fitness"""

    def __init__(self):
        super().__init__('fitness_evaluator')
        
        # Target position
        self.target_x = 10.0
        self.target_y = 10.0
        self.target_z = 5.0
        
        # Evaluation parameters
        self.start_time = None
        self.evaluation_duration = 30.0  # seconds
        self.current_pose = None
        self.pose_history = []
        self.velocity_history = []
        
        # Publishers
        self.fitness_pub = self.create_publisher(
            Float64,
            '/fitness/score',
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/quadrotor/odometry',
            self.odom_callback,
            10
        )
        
        self.collision_sub = self.create_subscription(
            Bool,
            '/collision/detected',
            self.collision_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # State
        self.collision_detected = False
        self.current_velocity = None
        self.evaluation_active = False
        
        # Timer untuk evaluasi
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Fitness Evaluator Node started')

    def odom_callback(self, msg):
        """Callback untuk odometry"""
        self.current_pose = msg.pose.pose
        self.pose_history.append(self.current_pose)
        
        # Keep only recent history
        if len(self.pose_history) > 1000:
            self.pose_history.pop(0)

    def cmd_vel_callback(self, msg):
        """Callback untuk command velocity"""
        vel = math.sqrt(
            msg.linear.x**2 + msg.linear.y**2 + msg.linear.z**2
        )
        self.current_velocity = vel
        self.velocity_history.append(vel)
        
        if len(self.velocity_history) > 1000:
            self.velocity_history.pop(0)

    def collision_callback(self, msg):
        """Callback untuk collision"""
        self.collision_detected = msg.data

    def calculate_distance_to_target(self):
        """Hitung jarak ke target"""
        if self.current_pose is None:
            return float('inf')
        
        dx = self.current_pose.position.x - self.target_x
        dy = self.current_pose.position.y - self.target_y
        dz = self.current_pose.position.z - self.target_z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def calculate_path_efficiency(self):
        """Hitung efisiensi path (jarak tempuh vs jarak langsung)"""
        if len(self.pose_history) < 2:
            return 1.0
        
        total_distance = 0.0
        for i in range(1, len(self.pose_history)):
            p1 = self.pose_history[i-1]
            p2 = self.pose_history[i]
            
            dx = p2.position.x - p1.position.x
            dy = p2.position.y - p1.position.y
            dz = p2.position.z - p1.position.z
            
            total_distance += math.sqrt(dx**2 + dy**2 + dz**2)
        
        if total_distance == 0:
            return 1.0
        
        direct_distance = self.calculate_distance_to_target()
        if direct_distance == 0:
            return 1.0
        
        return direct_distance / total_distance

    def calculate_stability(self):
        """Hitung stabilitas berdasarkan variasi velocity"""
        if len(self.velocity_history) < 10:
            return 1.0
        
        velocities = np.array(self.velocity_history[-100:])
        std_dev = np.std(velocities)
        
        # Lower std dev = more stable = higher score
        stability = 1.0 / (1.0 + std_dev)
        return stability

    def calculate_fitness(self):
        """Hitung total fitness score"""
        if self.current_pose is None:
            return 0.0
        
        # Penalty untuk collision
        if self.collision_detected:
            return -1000.0
        
        # Distance to target (closer = better)
        distance = self.calculate_distance_to_target()
        distance_score = 100.0 / (1.0 + distance)
        
        # Path efficiency
        path_efficiency = self.calculate_path_efficiency()
        
        # Stability
        stability = self.calculate_stability()
        
        # Combined fitness
        fitness = (
            distance_score * 0.5 +
            path_efficiency * 100.0 * 0.3 +
            stability * 100.0 * 0.2
        )
        
        return fitness

    def timer_callback(self):
        """Timer callback untuk publish fitness"""
        fitness = self.calculate_fitness()
        
        fitness_msg = Float64()
        fitness_msg.data = float(fitness)
        self.fitness_pub.publish(fitness_msg)
        
        self.get_logger().debug(f'Fitness: {fitness:.2f}')


def main(args=None):
    rclpy.init(args=args)
    
    evaluator = FitnessEvaluator()
    
    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        pass
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

