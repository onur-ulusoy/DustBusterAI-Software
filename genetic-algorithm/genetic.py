import random
import math
import matplotlib.pyplot as plt

# Define the cities and their coordinates
cities = ['A', 'B', 'C', 'D', 'E', 'F']
coords = {'A': (0, 0), 'B': (0, 5), 'C': (5, 5), 'D': (5, 0), 'E': (3, 3), 'F': (1, 1)}

# Calculate the distances between the cities
distances = {}
for i in range(len(cities)):
    for j in range(len(cities)):
        if i != j:
            distances[(cities[i], cities[j])] = math.sqrt((coords[cities[i]][0] - coords[cities[j]][0])**2 + (coords[cities[i]][1] - coords[cities[j]][1])**2)

# Define the genetic algorithm parameters
pop_size = 100
elite_size = 20
mutation_rate = 0.01
generations = 500

# Create an initial population of tours
def create_population(size):
    population = []
    for i in range(size):
        tour = random.sample(cities, len(cities))
        population.append(tour)
    return population

# Calculate the fitness of a tour
def fitness(tour):
    length = 0
    for i in range(len(tour)-1):
        length += distances[(tour[i], tour[i+1])]
    length += distances[(tour[-1], tour[0])]
    return 1/length

# Select the elite tours
def select_elite(population, size):
    ranked = [(fitness(tour), tour) for tour in population]
    ranked = sorted(ranked, reverse=True)
    elite = [ranked[i][1] for i in range(size)]
    return elite

# Crossover two parent tours to create a child tour
def crossover(parent1, parent2):
    child = [None] * len(parent1)
    start = random.randint(0, len(parent1)-1)
    end = random.randint(0, len(parent1)-1)
    if start < end:
        for i in range(start, end+1):
            child[i] = parent1[i]
    else:
        for i in range(end, start+1):
            child[i] = parent1[i]
    for i in range(len(parent2)):
        if parent2[i] not in child:
            for j in range(len(child)):
                if child[j] is None:
                    child[j] = parent2[i]
                    break
    return child

# Mutate a tour by swapping two cities
def mutate(tour):
    for i in range(len(tour)):
        if random.random() < mutation_rate:
            j = random.randint(0, len(tour)-1)
            tour[i], tour[j] = tour[j], tour[i]
    return tour

# Create the next generation of tours
def next_generation(population):
    elite = select_elite(population, elite_size)
    offspring = [crossover(random.choice(elite), random.choice(elite)) for i in range(pop_size-elite_size)]
    next_gen = elite + offspring
    next_gen = [mutate(tour) for tour in next_gen]
    return next_gen

# Solve the TSP problem using a genetic algorithm
def solve_tsp():
    # Create an initial population
    population = create_population(pop_size)
    best_tour = None
    best_fitness = 0
    # Iterate over the generations
    fitness_values = []
    for i in range(generations):
        # Calculate the fitness of each tour
        fitness_scores = [fitness(tour) for tour in population]
        max_fitness = max(fitness_scores)
        fitness_values.append(max_fitness)
        # Select the next generation
        population = next_generation(population)
        # Update the best tour if necessary
        if max_fitness > best_fitness:
            best_tour = population[fitness_scores.index(max_fitness)]
            best_fitness = max_fitness
    # Plot the fitness vs generations, optimal tour and total tour length vs generations graphs
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(6, 10))
    # Fitness vs Generations
    ax1.plot(fitness_values)
    ax1.set_xlabel('Generation')
    ax1.set_ylabel('Fitness')
    ax1.set_title('Fitness vs Generations')
    # Optimal Tour
    x = [coords[best_tour[i]][0] for i in range(len(best_tour))]
    x.append(coords[best_tour[0]][0])
    y = [coords[best_tour[i]][1] for i in range(len(best_tour))]
    y.append(coords[best_tour[0]][1])
    ax2.plot(x, y, 'ro-')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_title('Optimal Tour')
    # Total Tour Length vs Generations
    tour_lengths = [1/fitness for fitness in fitness_scores]
    ax3.plot(tour_lengths)
    ax3.set_xlabel('Generation')
    ax3.set_ylabel('Total Tour Length')
    ax3.set_title('Total Tour Length vs Generations')
    # Display the plots
    plt.tight_layout()
    plt.show()
    # Print the optimal tour and its length
    print('Optimal Tour:', best_tour)
    print('Tour Length:', 1/best_fitness)

def main():
    solve_tsp()

if __name__ == "__main__":
    main()