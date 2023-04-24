import random
import math
import matplotlib.pyplot as plt

class TSPGeneticSolver:
    def __init__(self, cities, coords, pop_size=100, elite_size=20, mutation_rate=0.01, generations=500):
        self.cities = cities
        self.coords = coords
        self.distances = {}
        for i in range(len(cities)):
            for j in range(len(cities)):
                if i != j:
                    self.distances[(cities[i], cities[j])] = math.sqrt(
                        (coords[cities[i]][0] - coords[cities[j]][0]) ** 2
                        + (coords[cities[i]][1] - coords[cities[j]][1]) ** 2
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
        for i in range(self.pop_size):
            tour = random.sample(self.cities, len(self.cities))
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
            self.next_generation()
            if max_fitness > best_fitness:
                best_tour = self.population[fitness_scores.index(max_fitness)]
                best_fitness = max_fitness
        return best_tour, 1/best_fitness, fitness_values

    def visualize(self, tour, fitness_values, fitness_scores):
        # Print the optimal tour and its length
        print('Optimal Tour:', tour)
        print('Tour Length:', 1 / fitness_scores[0])
        
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
            if i < len(self.cities):
                ax2.annotate(str(i + 1), (x_i, y_i), textcoords='offset points', xytext=(0, -4), ha='center', fontsize=12)

        # Total Tour Length vs Generations
        tour_lengths = [1 / fitness for fitness in fitness_scores]
        ax3.plot(tour_lengths)
        ax3.set_xlabel('Generation')
        ax3.set_ylabel('Total Tour Length')
        ax3.set_title('Total Tour Length vs Generations')

        # Display the plots
        plt.tight_layout()
        plt.show()


def main():
    cities = ['A', 'B', 'C', 'D', 'E', 'F', 'H', 'I', 'J', 'K', 'L', 'G']
    coords = {'A': (0, 0), 'B': (0, 5), 'C': (5, 5), 'D': (5, 0), 'E': (3, 3), 'F': (1, 1),
              'G': (4.25, 1.5), 'H': (0.78, 6), 'I': (2.2, 1), 'J': (5, 6), 'K': (3.3, 3), 'L': (0, 1)}
    
    solver = TSPGeneticSolver(cities, coords)
    solver.create_population()
    best_tour, tour_length, fitness_values = solver.solve()
    solver.visualize(best_tour, fitness_values, [solver.fitness(tour) for tour in solver.population])

if __name__=="__main__":
    main()