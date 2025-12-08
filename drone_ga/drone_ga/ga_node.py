#!/usr/bin/env python3
"""
GA Training Loop Node - FIX (Pose Reset Only)
Mencegah deadlock dengan menggunakan arsitektur non-blocking state machine.
Skip world reset untuk menjaga GPU rendering context (lidar sensors).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Bool
from nav_msgs.msg import Odometry
from ros_gz_interfaces.srv import ControlWorld
import time
import subprocess
import random
import numpy as np

STATE_IDLE = 0
STATE_RESETTING_POSE = 1
STATE_WAITING_SETTLE = 2
STATE_EPISODE_ACTIVE = 3

class GATrainingLoop(Node):
    
    def __init__(self):
        super().__init__('ga_training_loop')
        
        # Parameters
        self.declare_parameter('episode_timeout', 15.0)
        self.declare_parameter('population_size', 30)
        self.declare_parameter('max_generations', 100)
        self.declare_parameter('settling_time', 0.3)
        
        self.episode_timeout = self.get_parameter('episode_timeout').value
        self.population_size = self.get_parameter('population_size').value
        self.max_generations = self.get_parameter('max_generations').value
        self.settling_time = self.get_parameter('settling_time').value
        
        # GA state
        self.current_generation = 0
        self.current_individual = 0
        self.population = []
        self.fitness_scores = []
        self.best_weights = None
        self.best_fitness = -float('inf')
        
        # Simulation State Variables
        self.state = STATE_IDLE
        self.episode_start_time = None
        self.state_start_time = None
        self.collision_detected = False
        self.current_fitness = 0.0
        # self.reset_future = None  # Not used anymore (skip world reset)
        
        # Publishers
        self.weights_pub = self.create_publisher(
            Float64MultiArray, '/ga/weights', 10)
        
        self.reset_trigger_pub = self.create_publisher(
            Bool, '/ga/reset_trigger', 10)
        
        # Subscribers
        self.create_subscription(
            Float64, '/ga/fitness', self.fitness_callback, 10)
        
        self.create_subscription(
            Bool, '/drone/collision', self.collision_callback, 10)
            
        # Service Client for World Control
        self.world_control_client = self.create_client(
            ControlWorld, '/world/flappy/control')
            
        self.get_logger().info('Waiting for simulation to be ready...')

        self.control_timer = self.create_timer(0.1, self.training_loop)
        
        # Initialize population
        self.initialize_population()
        self.fitness_scores = [0.0] * self.population_size
        
        self.get_logger().info('=== GA Training Loop Started (Pose Reset Only - GPU Lidar Fix) ===')

    def initialize_population(self):
        """Initialize population with smart heuristics"""
        self.population = []
        
        # Smart seeds (20% of population) - based on intuition
        # Format: [lidar_up, lidar_down, altitude, bias]
        smart_seeds = [
            [0.5, -1.5, 0.3, 0.5],   # React to obstacles
            [1.0, -2.0, 0.5, 0.3],   # Aggressive avoidance
            [0.3, -1.0, 0.4, 0.7],   # Altitude focus
            [0.8, -1.8, 0.2, 0.4],   # Balanced
            [0.4, -1.2, 0.6, 0.5],   # Cautious
        ]
        
        num_seeds = min(len(smart_seeds), self.population_size // 5)
        for i in range(num_seeds):
            self.population.append(smart_seeds[i])
        
        # Random initialization for the rest
        while len(self.population) < self.population_size:
            weights = [random.uniform(-2.0, 2.0) for _ in range(4)]
            self.population.append(weights)
            
        self.get_logger().info(f'Initialized {self.population_size} individuals ({num_seeds} smart seeds)')

    def fitness_callback(self, msg):
        if self.state == STATE_EPISODE_ACTIVE:
            self.current_fitness = max(self.current_fitness, msg.data)

    def collision_callback(self, msg):
        if self.state == STATE_EPISODE_ACTIVE and msg.data:
            self.collision_detected = True

    # --- STATE MACHINE LOOP ---
    def training_loop(self):
        """Main State Machine Loop"""
        
        # Cek Ketersediaan Service (Hanya untuk awal, tidak blocking)
        # World control service tidak lagi digunakan untuk reset
        # tapi kita tetap cek untuk memastikan simulasi sudah ready
        if not self.world_control_client.service_is_ready():
            return

        if self.state == STATE_IDLE:
            if self.current_generation >= self.max_generations:
                self.get_logger().info('TRAINING COMPLETE!')
                self.control_timer.cancel()
                return

            if self.current_individual >= self.population_size:
                self.evolve_generation()
                return

            # PERBAIKAN: Langsung ke pose reset, skip world reset
            # World reset merusak GPU rendering context untuk lidar
            self.trigger_pose_reset()

        elif self.state == STATE_RESETTING_POSE:
            # Beri sedikit waktu setelah subprocess call (0.5 detik)
            now = self.get_clock().now()
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed >= 0.5:
                self.state = STATE_WAITING_SETTLE
                self.state_start_time = now

        elif self.state == STATE_WAITING_SETTLE:
            now = self.get_clock().now()
            elapsed = (now - self.state_start_time).nanoseconds / 1e9
            if elapsed >= self.settling_time:
                # Kirim sinyal reset ke node controller/fitness
                reset_msg = Bool()
                reset_msg.data = True
                self.reset_trigger_pub.publish(reset_msg)
                
                # Mulai Episode
                self.start_episode()
                self.state = STATE_EPISODE_ACTIVE

        elif self.state == STATE_EPISODE_ACTIVE:
            now = self.get_clock().now()
            elapsed = (now - self.episode_start_time).nanoseconds / 1e9
            
            if elapsed > self.episode_timeout or self.collision_detected:
                reason = "Collision" if self.collision_detected else "Timeout"
                self.end_episode(reason)
                self.state = STATE_IDLE # Kembali ke awal untuk individu berikutnya

    def trigger_pose_reset(self):
        """Reset pose drone saja (skip world reset untuk menjaga GPU rendering context)"""
        self.get_logger().info(f'Gen {self.current_generation+1} Ind {self.current_individual+1}: Resetting Pose...')
        self.reset_drone_pose()
        self.state = STATE_RESETTING_POSE
        self.state_start_time = self.get_clock().now()

    def reset_drone_pose(self):
        """Reset pose via subprocess (ign service CLI)"""
        try:
            cmd = [
                'ign', 'service',
                '-s', '/world/flappy/set_pose',
                '--reqtype', 'ignition.msgs.Pose',
                '--reptype', 'ignition.msgs.Boolean',
                '--timeout', '5000',
                '--req', 'name: "quadrotor" position: {x: 0.0 y: 0.0 z: 6.0} orientation: {w: 1.0}'
            ]
            subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)
        except Exception as e:
            self.get_logger().error(f'Failed to reset pose CLI: {e}')

    def start_episode(self):
        """Setup variabel untuk episode baru"""
        weights = self.population[self.current_individual]
        
        # Publish weights
        weights_msg = Float64MultiArray()
        weights_msg.data = weights
        self.weights_pub.publish(weights_msg)
        
        # Reset variable episode
        self.collision_detected = False
        self.current_fitness = 0.0
        self.episode_start_time = self.get_clock().now()
        
        self.get_logger().info(f'  >> Episode Started. Weights: {[f"{w:.2f}" for w in weights]}')

    def end_episode(self, reason):
        """Simpan fitness dan log hasil"""
        self.fitness_scores[self.current_individual] = self.current_fitness
        
        # Update Best
        if self.current_fitness > self.best_fitness:
            self.best_fitness = self.current_fitness
            self.best_weights = self.population[self.current_individual].copy()
            self.get_logger().info(f'  🏆 NEW BEST: {self.best_fitness:.2f}')
            
        self.get_logger().info(f'  << Ended ({reason}). Fitness: {self.current_fitness:.2f}')
        self.current_individual += 1

    def evolve_generation(self):
        """Algoritma Genetika: Seleksi, Crossover, Mutasi"""
        self.get_logger().info('='*40)
        self.get_logger().info(f'GENERATION {self.current_generation + 1} COMPLETE')
        self.get_logger().info(f'Avg Fitness: {np.mean(self.fitness_scores):.2f}')
        self.get_logger().info(f'Max Fitness: {np.max(self.fitness_scores):.2f}')
        self.get_logger().info('='*40)
        
        new_pop = []
        
        # Elitism (Keep top 3 - increased from 2)
        indices = np.argsort(self.fitness_scores)[::-1]
        for i in range(min(3, self.population_size)):
            new_pop.append(self.population[indices[i]].copy())
        
        # Helper: Tournament Selection (increased selectivity)
        def tournament():
            k = 4  # Tournament size 3 -> 4
            candidates = random.sample(list(enumerate(self.fitness_scores)), k)
            best = max(candidates, key=lambda x: x[1])
            return self.population[best[0]]

        # Generate sisa populasi
        while len(new_pop) < self.population_size:
            p1 = tournament()
            p2 = tournament()
            
            # Crossover (increased probability 0.7 -> 0.8)
            if random.random() < 0.8:
                pt = random.randint(1, 3)
                c1 = p1[:pt] + p2[pt:]
                c2 = p2[:pt] + p1[pt:]
            else:
                c1, c2 = p1.copy(), p2.copy()
            
            # Mutation (increased rate 0.2 -> 0.35, adaptive strength)
            for c in [c1, c2]:
                if random.random() < 0.35:
                    idx = random.randint(0, 3)
                    # Adaptive mutation: stronger early, weaker later
                    strength = 0.8 * (1.0 - self.current_generation / self.max_generations) + 0.2
                    c[idx] += random.gauss(0, strength)
                    c[idx] = np.clip(c[idx], -3.0, 3.0)
                new_pop.append(c)
                if len(new_pop) >= self.population_size:
                    break
        
        self.population = new_pop
        self.fitness_scores = [0.0] * self.population_size
        self.current_individual = 0
        self.current_generation += 1
        self.state = STATE_IDLE # Siap untuk loop berikutnya

def main(args=None):
    rclpy.init(args=args)
    node = GATrainingLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()