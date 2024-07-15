"""
test_ga_no_thread.py

 This script contains unit tests for a basic Genetic Algorithm (GA) implementation using the unittest framework.
 It evaluates the evolution of a population of creatures represented by genetic data through simulated environments.

 The purpose of this code is to verify the functionality of the GA process, including fitness evaluation, selection, crossover,
 mutation, and elitism. The tests assess if the GA correctly evolves creatures over iterations to optimize their performance.

 It is noted that on certain environments (Windows with any Python version, M1 Mac with any Python version, or Intel Mac
 with Python > 3.7), the multi-threaded version of the simulation does not function correctly. Hence, this version uses
 a single-threaded Simulation instance instead of the multi-threaded ThreadedSim.

 The TestGA class defines several test cases:
 - testBasicGA: Implements a basic GA loop where each creature in a population is evaluated using a non-threaded Simulation,
                followed by selection, crossover, mutation, and elitism steps. Fitness metrics and genetic diversity are printed
                at each iteration to track progress. The final assertion ensures that the fittest creature's fitness is not zero.

"""
import unittest
import population
import simulation
import genome
import creature
import numpy as np
import pandas as pd  # Import for CSV handling
import os  # Import for creating directories

class TestGA(unittest.TestCase):
    def testBasicGA(self):
        """
        Test case for basic Genetic Algorithm (GA) implementation.
        """
        pop = population.Population(pop_size=10, gene_count=3)
        sim = simulation.Simulation()  # Use single-threaded Simulation due to platform-specific limitations

        # Initialize an empty list to store the data for each generation
        data = []

        ga_generations = 1000  # Define the number of generations

        # Create 'data' directory if it doesn't exist
        os.makedirs('data', exist_ok=True)

        for iteration in range(ga_generations):
            # Run the simulation for each creature in the population
            total_vertical_distances = []
            total_joins = []
            for cr in pop.creatures:
                sim.run_creature(cr, 2400)
                total_vertical_distances.append(cr.get_total_vertical_distance_travelled())
                total_joins.append(len(cr.get_expanded_links()))
            
            # Collect fitness and genetic information
            fits = [cr.get_total_vertical_distance_travelled() for cr in pop.creatures]
            links = [len(cr.get_expanded_links()) for cr in pop.creatures]
            
            # Print metrics for monitoring progress
            print(iteration, "fittest:", np.round(np.max(fits), 3),
                  "mean:", np.round(np.mean(fits), 3), "mean links", np.round(np.mean(links)),
                  "max links", np.round(np.max(links)), "mean joins:", np.round(np.mean(total_joins)),
                  "max joins:", np.round(np.max(total_joins)), "max vertical distance:",
                  np.round(np.max(total_vertical_distances), 3), "mean vertical distance:",
                  np.round(np.mean(total_vertical_distances), 3))
            
            # Store the data for this generation in a dictionary
            generation_data = {
                "iteration": iteration,
                "fittest": np.round(np.max(fits), 3),
                "mean": np.round(np.mean(fits), 3),
                "mean links": np.round(np.mean(links)),
                "max links": np.round(np.max(links)),
                "mean joins": np.round(np.mean(total_joins)),
                "max joins": np.round(np.max(total_joins)),
                "max vertical distance": np.round(np.max(total_vertical_distances), 3),
                "mean vertical distance": np.round(np.mean(total_vertical_distances), 3),
            }
    
            # Add the dictionary to the list
            data.append(generation_data)

            # Create a fitness map and evolve the population
            fit_map = population.Population.get_fitness_map(fits)
            new_creatures = []

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
                if cr.get_total_vertical_distance_travelled() == max_fit:
                    new_cr = creature.Creature(1)
                    new_cr.update_dna(cr.dna)
                    new_creatures[0] = new_cr
                    if iteration % 10 == 0:  # Save every 10th generation
                        filename = os.path.join("data", f"elite_{iteration}.csv")
                        genome.Genome.to_csv(cr.dna, filename)
                    break

            # Update the population with new creatures
            pop.creatures = new_creatures

        # Convert the list of dictionaries to a DataFrame
        df = pd.DataFrame(data)

        # Save the DataFrame to a CSV file
        df.to_csv("ga_output.csv", index=False)

        # Assertion: Ensure that the fittest creature's fitness is not zero
        self.assertNotEqual(fits[0], 0)

# Run all the test cases when the script is executed
if __name__ == "__main__":
    unittest.main()
