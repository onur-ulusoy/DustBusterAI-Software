import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import matplotlib.pyplot as plt
import numpy as np
import math

class TSPGeneticSolver:

    def __init__(self, coords, pop_size=100, elite_size=20, mutation_rate=0.01, generations=500):
        self.coords = coords
        self.distances = {}
        num_coords = len(coords)
        for i in range(num_coords):
            for j in range(num_coords):
                if i != j:
                    self.distances[(i, j)] = math.sqrt(
                        (coords[i][0] - coords[j][0]) ** 2
                        + (coords[i][1] - coords[j][1]) ** 2
                    )
        self.pop_size = pop_size
        self.elite_size = elite_size
        self.mutation_rate = mutation_rate
        self.generations = generations
        self.best_tour = None
        self.best_fitness = 0
        self.fitness_values = []
        self.population = None

    def create_population(self):
        population = []
        indices = list(range(len(self.coords)))
        for i in range(self.pop_size):
            tour = random.sample(indices, len(self.coords))
            population.append(tour)
        self.population = population
        return population

    def fitness(self, tour):
        length = 0
        for i in range(len(tour) - 1):
            length += self.distances[(tour[i], tour[i + 1])]
        length += self.distances[(tour[-1], tour[0])]
        return 1 / length

    def select_elite(self):
        ranked = [(self.fitness(tour), tour) for tour in self.population]
        ranked = sorted(ranked, reverse=True)
        elite = [ranked[i][1] for i in range(self.elite_size)]
        return elite

    def crossover(self, parent1, parent2):
        child = [None] * len(parent1)
        start = random.randint(0, len(parent1) - 1)
        end = random.randint(0, len(parent1) - 1)
        if start < end:
            for i in range(start, end + 1):
                child[i] = parent1[i]
        else:
            for i in range(end, start + 1):
                child[i] = parent1[i]
        for i in range(len(parent2)):
            if parent2[i] not in child:
                for j in range(len(child)):
                    if child[j] is None:
                        child[j] = parent2[i]
                        break
        return child

    def mutate(self, tour):
        for i in range(len(tour)):
            if random.random() < self.mutation_rate:
                j = random.randint(0, len(tour) - 1)
                tour[i], tour[j] = tour[j], tour[i]
        return tour

    def next_generation(self):
        elite = self.select_elite()
        offspring = [self.crossover(random.choice(elite), random.choice(elite)) for i in range(self.pop_size - self.elite_size)]
        next_gen = elite + offspring
        next_gen = [self.mutate(tour) for tour in next_gen]
        self.population = next_gen

    def solve(self):
        self.create_population()
        best_tour = None
        best_fitness = 0
        fitness_values = []
        for i in range(self.generations):
            fitness_scores = [self.fitness(tour) for tour in self.population]
            max_fitness = max(fitness_scores)
            fitness_values.append(max_fitness)
            if max_fitness > best_fitness:
                best_fitness = max_fitness
                best_tour = self.population[fitness_scores.index(max_fitness)]

            self.next_generation()

        self.best_tour = best_tour
        self.best_fitness = best_fitness
        self.fitness_values = fitness_values
        return best_tour, 1 / best_fitness, fitness_values

    def visualize(self, tour, fitness_values):
        # Print the optimal tour and its length
        print('Optimal Tour:', tour)
        print('Tour Length:', 1 / self.best_fitness)

        # Plot the fitness vs generations, optimal tour and total tour length vs generations graphs
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(6, 10))

        # Fitness vs Generations
        ax1.plot(fitness_values)
        ax1.set_xlabel('Generation')
        ax1.set_ylabel('Fitness')
        ax1.set_title('Fitness vs Generations')

        # Optimal Tour
        x = [self.coords[tour[i]][0] for i in range(len(tour))]
        x.append(self.coords[tour[0]][0])
        y = [self.coords[tour[i]][1] for i in range(len(tour))]
        y.append(self.coords[tour[0]][1])
        ax2.plot(x, y, 'ro-')
        ax2.set_xlabel('X')
        ax2.set_ylabel('Y')
        ax2.set_title('Optimal Tour')

        # Add numbers to the Optimal Tour graph
        for i, (x_i, y_i) in enumerate(zip(x, y)):
            if i < len(self.coords):
                ax2.annotate(str(i + 1), (x_i, y_i), textcoords='offset points', xytext=(0, -4), ha='center', fontsize=12)

        # Total Tour Length vs Generations
        tour_lengths = [1 / fitness for fitness in fitness_values]
        ax3.plot(tour_lengths)
        ax3.set_xlabel('Generation')
        ax3.set_ylabel('Total Tour Length')
        ax3.set_title('Total Tour Length vs Generations')

        # Display the plots
        plt.tight_layout()
        plt.show()

    def visualize_optimal_tour(self, tour):
        # Print the optimal tour and its length
        print('Optimal Tour:', tour)
        print('Tour Length:', 1 / self.best_fitness)

        # Optimal Tour
        fig, ax = plt.subplots(figsize=(6, 6))
        x = [self.coords[tour[i]][0] for i in range(len(tour))]
        x.append(self.coords[tour[0]][0])
        y = [self.coords[tour[i]][1] for i in range(len(tour))]
        y.append(self.coords[tour[0]][1])
        ax.plot(x, y, 'ro-')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_title('Optimal Tour')

        # Add numbers to the Optimal Tour graph
        for i, (x_i, y_i) in enumerate(zip(x, y)):
            if i < len(self.coords):
                ax.annotate(str(i + 1), (x_i, y_i), textcoords='offset points', xytext=(0, -4), ha='center', fontsize=12)

        # Display the plot
        plt.tight_layout()
        plt.show()

    def get_optimal_tour_points(self, tour):
        # Optimal Tour
        x = [self.coords[tour[i]][0] for i in range(len(tour))]
        x.append(self.coords[tour[0]][0])
        y = [self.coords[tour[i]][1] for i in range(len(tour))]
        y.append(self.coords[tour[0]][1])

        return x,y

class TSPSolverNode(Node):
    def __init__(self):
        super().__init__('tsp_solver_node')
        self.publisher_ = self.create_publisher(String, 'tsp_optimal_tour', 10)

    def publish_optimal_tour(self):
        pgm_file = "/home/onur/Desktop/DustBusterAI-Software/dustbuster_ws/src/dustbuster_navigation/dustbuster_navigation/map2.pgm"
        sample_rate = 30
        origin_x = -2.9
        origin_y = -4.05

        image = load_pgm_image(pgm_file)
        coords = discretize_walkable_area(image, sample_rate)
        resolution = 0.05
        origin = (origin_x, origin_y)

        points = return_points(coords, image.shape, sample_rate, resolution, origin)

        solver = TSPGeneticSolver(points, pop_size=100, elite_size=20, mutation_rate=0.01, generations=800)
        best_tour, best_length, fitness_values = solver.solve()
        x, y = solver.get_optimal_tour_points(best_tour)

        msg = String()
        msg.data = ';'.join([f"{xi}, {yi}" for xi, yi in zip(x, y)])
        self.publisher_.publish(msg)
        solver.visualize_optimal_tour(best_tour)


import os, sys
module_dir = os.path.abspath("/home/onur/Desktop/DustBusterAI-Software/dustbuster_ws/src/dustbuster_navigation/dustbuster_navigation")
sys.path.insert(0, module_dir)

from map_discretizer import *
import save_map
import time

def main(args=None):
    # Save the current map
    map_dir = '/home/onur/Desktop/DustBusterAI-Software/dustbuster_ws/src/dustbuster_navigation/dustbuster_navigation'
    map_name = 'map2'
    map_params = ['--map_path', map_dir, map_name]
    save_map.main(map_params)
    time.sleep(2)

    rclpy.init(args=args)
    tsp_solver_node = TSPSolverNode()

    # Call the publish_optimal_tour method directly after creating the node instance
    tsp_solver_node.publish_optimal_tour()

    # No need to spin the node since we only want to publish once
    tsp_solver_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
