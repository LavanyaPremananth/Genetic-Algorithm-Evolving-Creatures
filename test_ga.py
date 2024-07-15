"""

test_ga.py 

This script contains unit tests for a basic Genetic Algorithm (GA) implementation using the unittest framework.
It evaluates the evolution of a population of creatures represented by genetic data through simulated environments.

The purpose of this code is to verify the functionality of the GA process, including fitness evaluation, selection, crossover,
mutation, and elitism. The tests assess if the GA correctly evolves creatures over iterations to optimize their performance.
 
It is noted that on certain environments (Windows with any Python version, M1 Mac with any Python version, or Intel Mac
with Python > 3.7), the multi-threaded version of the simulation does not function correctly. Hence, this version uses
a single-threaded ThreadedSim instance instead of the multi-threaded Simulation, as specified in the comments.

The TestGA class defines a single test case:
- testBasicGA: Implements a basic GA loop where each creature in a population is evaluated using a multi-threaded Simulation,
               followed by selection, crossover, mutation, and elitism steps. Fitness metrics and genetic diversity are printed
               at each iteration to track progress. The final assertion ensures that the fittest creature's fitness is not zero.

"""

import unittest
import population
import simulation
import genome
import creature
import numpy as np

class TestGA(unittest.TestCase):
    def testBasicGA(self):
        """
        Test case for basic Genetic Algorithm (GA) implementation.
        """
        pop = population.Population(pop_size=10, gene_count=3)
        sim = simulation.ThreadedSim(pool_size=1)  # Use single-threaded ThreadedSim due to platform-specific limitations

        for iteration in range(1000):
            # Run the simulation for the entire population
            sim.eval_population(pop, 2400)
            
            # Collect fitness and genetic information
            fits = [cr.get_distance_travelled() for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]
            
            # Print metrics for monitoring progress
            print(iteration, "fittest:", np.round(np.max(fits), 3),
                  "mean:", np.round(np.mean(fits), 3), "mean links", np.round(np.mean(links)), "max links", np.round(np.max(links)))
            
            # Create a fitness map and evolve the population
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []
            
            # Generate new creatures through selection, crossover, mutation, and elitism
            for i in range(len(pop.creatures)):
                p1_ind = population.Population.select_parent(fit_map)
                p2_ind = population.Population.select_parent(fit_map)
                p1 = pop.creatures[p1_ind]
                p2 = pop.creatures[p2_ind]
                
                # Perform genetic operations
                dna = genome.Genome.crossover(p1.dna, p2.dna)
                dna = genome.Genome.point_mutate(dna, rate=0.1, amount=0.25)
                dna = genome.Genome.shrink_mutate(dna, rate=0.25)
                dna = genome.Genome.grow_mutate(dna, rate=0.1)
                
                # Create a new creature and add to the new population
                cr = creature.Creature(1)
                cr.update_dna(dna)
                new_creatures.append(cr)
            
            # Elitism: Preserve the top performer from the previous generation
            max_fit = np.max(fits)
            for cr in pop.creatures:
                if cr.get_distance_travelled() == max_fit:
                    new_cr = creature.Creature(1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    filename = "elite_"+str(iteration)+".csv"
                    genome.Genome.to_csv(cr.dna, filename)
                    break
            
            # Update the population with new creatures
            pop.creatures = new_creatures
        
        # Assertion: Ensure that the fittest creature's fitness is not zero
        self.assertNotEqual(fits[0], 0)
    
    def testGASettings(self):
        """
        Test different Genetic Algorithm settings to observe their impact on parent selection.
        """
        # Define different population sizes, mutation rates, and crossover rates to test
        population_sizes = [10, 20, 50]
        mutation_rates = [0.01, 0.05, 0.1]
        crossover_rates = [0.6, 0.7, 0.8]

        # Iterate over different settings
        for pop_size in population_sizes:
            for mutation_rate in mutation_rates:
                for crossover_rate in crossover_rates:
                    # Initialize a population with these settings
                    pop = population.Population(pop_size=pop_size, gene_count=3)
                    sim = simulation.ThreadedSim(pool_size=1)

                    for iteration in range(100):
                        # Run the simulation for the entire population
                        sim.eval_population(pop, 2400)

                        # Collect fitness and genetic information
                        fits = [cr.get_distance_travelled() for cr in pop.creatures]
                        links = [len(cr.get_expanded_links()) for cr in pop.creatures]

                        # Print metrics for monitoring progress
                        print(f"Pop Size: {pop_size}, Mutation Rate: {mutation_rate}, Crossover Rate: {crossover_rate}, Iteration: {iteration}, Fittest: {np.round(np.max(fits), 3)}, Mean: {np.round(np.mean(fits), 3)}, Mean Links: {np.round(np.mean(links), 3)}, Max Links: {np.round(np.max(links), 3)}")

                        # Create a fitness map and evolve the population
                        fit_map = population.Population.get_fitness_map(fits)
                        new_creatures = []

                        # Generate new creatures through selection, crossover, mutation, and elitism
                        for i in range(len(pop.creatures)):
                            p1_ind = population.Population.select_parent(fit_map)
                            p2_ind = population.Population.select_parent(fit_map)
                            p1 = pop.creatures[p1_ind]
                            p2 = pop.creatures[p2_ind]

                            # Perform genetic operations
                            dna = genome.Genome.crossover(p1.dna, p2.dna)
                            dna = genome.Genome.point_mutate(dna, rate=mutation_rate, amount=0.25)
                            dna = genome.Genome.shrink_mutate(dna, rate=0.25)
                            dna = genome.Genome.grow_mutate(dna, rate=mutation_rate)

                            # Create a new creature and add to the new population
                            cr = creature.Creature(1)
                            cr.update_dna(dna)
                            new_creatures.append(cr)

                        # Elitism: Preserve the top performer from the previous generation
                        max_fit = np.max(fits)
                        for cr in pop.creatures:
                            if cr.get_distance_travelled() == max_fit:
                                new_cr = creature.Creature(1)
                                new_cr.update_dna(cr.dna)
                                new_creatures[0] = new_cr
                                break

                        # Update the population with new creatures
                        pop.creatures = new_creatures
        
        # Assertion: Ensure that the fittest creature's fitness is not zero
        self.assertNotEqual(fits[0], 0)

# Run all the test cases when the script is executed
if __name__ == '__main__':
    unittest.main()