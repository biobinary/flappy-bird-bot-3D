#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Int32
import numpy as np
import random
import time

class DroneGANode(Node):

    def __init__(self):

        super().__init__('drone_ga')

        self.population_size = 20
        self.mutation_rate = 0.1
        self.num_weights = 4
        self.max_generations = 100
        
        self.current_generation = 1
        self.current_individual_idx = 0
        self.population = []
        self.fitness_scores = []
        self.waiting_for_fitness = False
        
        self.initialize_population()

        self.weights_pub = self.create_publisher(
            Float64MultiArray, 
            '/ga/weights', 
            10
        )
        
        self.best_weights_pub = self.create_publisher(
            Float64MultiArray, 
            '/ga/best_weights', 
            10
        )
        
        self.gen_pub = self.create_publisher(
            Int32,
            '/ga/generation',
            10
        )

        self.fitness_sub = self.create_subscription(
            Float64,
            '/ga/fitness',
            self.fitness_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.game_loop)
        
        self.get_logger().info('Drone GA Node Started. Ready to evolve!')

    def initialize_population(self):

        """Membuat populasi awal dengan bobot random."""
        
        self.population = []
        for _ in range(self.population_size):
            individual = [random.uniform(-1.0, 1.0) for _ in range(self.num_weights)]
            self.population.append(individual)
        
        self.fitness_scores = [0.0] * self.population_size

    def fitness_callback(self, msg):

        """Callback saat menerima fitness dari evaluator."""
        
        if self.waiting_for_fitness:
            score = msg.data
            self.fitness_scores[self.current_individual_idx] = score
            
            self.get_logger().info(
                f'Gen {self.current_generation} | Indiv {self.current_individual_idx + 1}/{self.population_size} | Fitness: {score:.2f}'
            )
            
            self.current_individual_idx += 1
            self.waiting_for_fitness = False

    def publish_current_weights(self):
        """Mengirim bobot individu saat ini ke topik /ga/weights."""
        weights = self.population[self.current_individual_idx]
        
        msg = Float64MultiArray()
        msg.data = weights
        self.weights_pub.publish(msg)
        
        self.get_logger().info(f'Testing Individual {self.current_individual_idx + 1}: {weights}')
        self.waiting_for_fitness = True

    def game_loop(self):
        """Loop utama logika GA."""
        
        if self.waiting_for_fitness:
            return

        if self.current_individual_idx < self.population_size:
            self.publish_current_weights()
        else:
            self.evolve_generation()

    def selection_tournament(self):

        """Melakukan Tournament Selection."""
        
        tournament_size = 3
        candidates_idx = random.sample(range(self.population_size), tournament_size)
        
        best_idx = candidates_idx[0]
        for idx in candidates_idx:
            if self.fitness_scores[idx] > self.fitness_scores[best_idx]:
                best_idx = idx
                
        return self.population[best_idx]

    def crossover(self, parent1, parent2):
        
        """Single Point Crossover."""
        
        point = random.randint(1, self.num_weights - 1)
        
        child1 = parent1[:point] + parent2[point:]
        child2 = parent2[:point] + parent1[point:]
        
        return child1, child2

    def mutate(self, individual):
        
        """Mutasi gen dengan probabilitas tertentu."""
        
        mutated_indiv = list(individual) # Copy
        
        for i in range(len(mutated_indiv)):
            if random.random() < self.mutation_rate:
                mutated_indiv[i] += random.gauss(0, 0.2)
                mutated_indiv[i] = max(min(mutated_indiv[i], 5.0), -5.0)

        return mutated_indiv

    def evolve_generation(self):

        """Membentuk generasi baru."""
        
        best_fitness_idx = np.argmax(self.fitness_scores)
        best_weights = self.population[best_fitness_idx]
        best_score = self.fitness_scores[best_fitness_idx]

        best_msg = Float64MultiArray()
        best_msg.data = best_weights
        self.best_weights_pub.publish(best_msg)
        
        gen_msg = Int32()
        gen_msg.data = self.current_generation
        self.gen_pub.publish(gen_msg)

        self.get_logger().info('=============================================')
        self.get_logger().info(f'GENERATION {self.current_generation} COMPLETE')
        self.get_logger().info(f'Best Fitness: {best_score:.2f}')
        self.get_logger().info(f'Best Weights: {best_weights}')
        self.get_logger().info('=============================================')

        new_population = []

        new_population.append(best_weights)
        new_population.append(best_weights)

        while len(new_population) < self.population_size:

            p1 = self.selection_tournament()
            p2 = self.selection_tournament()
            
            c1, c2 = self.crossover(p1, p2)
            
            c1 = self.mutate(c1)
            c2 = self.mutate(c2)
            
            new_population.append(c1)
            if len(new_population) < self.population_size:
                new_population.append(c2)

        self.population = new_population
        self.fitness_scores = [0.0] * self.population_size
        self.current_individual_idx = 0
        self.current_generation += 1
        
        self.get_logger().info(f'Starting Generation {self.current_generation}...')

def main(args=None):
    rclpy.init(args=args)
    
    node = DroneGANode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()