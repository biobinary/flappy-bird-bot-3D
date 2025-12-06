#!/usr/bin/env python3
"""
Genetic Algorithm Node
Mengimplementasikan Genetic Algorithm untuk optimasi kontrol drone
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Bool
import numpy as np
import random


class GeneticAlgorithmNode(Node):
    """Node untuk Genetic Algorithm optimasi"""

    def __init__(self):
        super().__init__('genetic_algorithm')
        
        # GA Parameters
        self.population_size = 50
        self.generation = 0
        self.max_generations = 100
        self.mutation_rate = 0.1
        self.crossover_rate = 0.7
        
        # Population: setiap individu adalah parameter kontrol
        # Format: [Kp_x, Kp_y, Kp_z, Kd_x, Kd_y, Kd_z, ...]
        self.population = []
        self.fitness_scores = []
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/drone/cmd_vel',
            10
        )
        
        self.best_fitness_pub = self.create_publisher(
            Float64,
            '/ga/best_fitness',
            10
        )
        
        self.generation_pub = self.create_publisher(
            Float64,
            '/ga/generation',
            10
        )
        
        # Subscribers
        self.fitness_sub = self.create_subscription(
            Float64,
            '/fitness/score',
            self.fitness_callback,
            10
        )
        
        self.collision_sub = self.create_subscription(
            Bool,
            '/collision/detected',
            self.collision_callback,
            10
        )
        
        # State
        self.current_fitness = 0.0
        self.collision_detected = False
        self.current_individual = None
        self.evaluation_active = False
        
        # Initialize population
        self.initialize_population()
        
        # Timer untuk GA evolution
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info('Genetic Algorithm Node started')
        self.get_logger().info(f'Population size: {self.population_size}')

    def initialize_population(self):
        """Initialize random population"""
        self.population = []
        for _ in range(self.population_size):
            # Random parameters untuk PID controller
            individual = {
                'Kp': [random.uniform(0.1, 2.0) for _ in range(3)],  # x, y, z
                'Kd': [random.uniform(0.01, 0.5) for _ in range(3)],
                'Ki': [random.uniform(0.0, 0.1) for _ in range(3)],
            }
            self.population.append(individual)
        
        self.fitness_scores = [0.0] * self.population_size

    def fitness_callback(self, msg):
        """Callback untuk menerima fitness score"""
        if self.evaluation_active and self.current_individual is not None:
            self.current_fitness = msg.data
            self.get_logger().info(f'Fitness received: {self.current_fitness}')

    def collision_callback(self, msg):
        """Callback untuk collision detection"""
        self.collision_detected = msg.data
        if self.collision_detected:
            self.get_logger().warn('Collision detected!')

    def evaluate_individual(self, individual):
        """Evaluate fitness untuk satu individu"""
        # Set controller parameters
        # Publish command untuk testing
        # Wait for fitness evaluation
        self.current_individual = individual
        self.evaluation_active = True
        self.current_fitness = 0.0
        
        # Generate test command
        cmd = Twist()
        cmd.linear.x = 1.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.5
        self.cmd_vel_pub.publish(cmd)
        
        # Return fitness (akan diupdate via callback)
        return self.current_fitness

    def selection(self):
        """Tournament selection"""
        tournament_size = 3
        selected = []
        
        for _ in range(self.population_size):
            tournament = random.sample(
                list(zip(self.population, self.fitness_scores)),
                tournament_size
            )
            winner = max(tournament, key=lambda x: x[1])
            selected.append(winner[0])
        
        return selected

    def crossover(self, parent1, parent2):
        """Single point crossover"""
        if random.random() > self.crossover_rate:
            return parent1.copy(), parent2.copy()
        
        child1 = {}
        child2 = {}
        
        for key in parent1.keys():
            if isinstance(parent1[key], list):
                point = random.randint(1, len(parent1[key]) - 1)
                child1[key] = parent1[key][:point] + parent2[key][point:]
                child2[key] = parent2[key][:point] + parent1[key][point:]
            else:
                child1[key] = parent1[key]
                child2[key] = parent2[key]
        
        return child1, child2

    def mutate(self, individual):
        """Mutation operator"""
        mutated = individual.copy()
        
        for key in mutated.keys():
            if isinstance(mutated[key], list):
                for i in range(len(mutated[key])):
                    if random.random() < self.mutation_rate:
                        if key == 'Kp':
                            mutated[key][i] = random.uniform(0.1, 2.0)
                        elif key == 'Kd':
                            mutated[key][i] = random.uniform(0.01, 0.5)
                        elif key == 'Ki':
                            mutated[key][i] = random.uniform(0.0, 0.1)
        
        return mutated

    def evolve(self):
        """Evolve population untuk satu generation"""
        # Evaluate all individuals
        for i, individual in enumerate(self.population):
            fitness = self.evaluate_individual(individual)
            self.fitness_scores[i] = fitness
        
        # Find best
        best_idx = np.argmax(self.fitness_scores)
        best_fitness = self.fitness_scores[best_idx]
        best_individual = self.population[best_idx]
        
        self.get_logger().info(
            f'Generation {self.generation}: Best fitness = {best_fitness}'
        )
        
        # Publish best fitness
        fitness_msg = Float64()
        fitness_msg.data = float(best_fitness)
        self.best_fitness_pub.publish(fitness_msg)
        
        gen_msg = Float64()
        gen_msg.data = float(self.generation)
        self.generation_pub.publish(gen_msg)
        
        # Selection
        selected = self.selection()
        
        # Crossover and mutation
        new_population = [best_individual]  # Elitism
        
        while len(new_population) < self.population_size:
            parent1 = random.choice(selected)
            parent2 = random.choice(selected)
            
            child1, child2 = self.crossover(parent1, parent2)
            child1 = self.mutate(child1)
            child2 = self.mutate(child2)
            
            new_population.append(child1)
            if len(new_population) < self.population_size:
                new_population.append(child2)
        
        self.population = new_population[:self.population_size]
        self.generation += 1

    def timer_callback(self):
        """Timer callback untuk GA evolution"""
        if self.generation < self.max_generations:
            if not self.evaluation_active:
                self.evolve()
        else:
            self.get_logger().info('GA completed!')
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    
    ga_node = GeneticAlgorithmNode()
    
    try:
        rclpy.spin(ga_node)
    except KeyboardInterrupt:
        pass
    finally:
        ga_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

