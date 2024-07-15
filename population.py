"""
population.py 

This Population class is designed to manage a population of creatures, 
where each creature has a set of genetic parameters (gene_count). 

It provides methods to initialize a population (__init__), 
generate a cumulative fitness map based on fitness values (get_fitness_map), 
and select a parent index based on the fitness map using stochastic universal sampling (select_parent). 
"""

import creature 
import numpy as np

class Population:
    def __init__(self, pop_size, gene_count):
        """
        Initializes a population of creatures with random genetic parameters.

        Args:
        - pop_size (int): Number of creatures in the population.
        - gene_count (int): Number of genes per creature.
        """
        self.creatures = [creature.Creature(gene_count=gene_count) for _ in range(pop_size)]

    @staticmethod
    def get_fitness_map(fits):
        """
        Generates a cumulative fitness map based on the fitness values provided.

        Args:
        - fits (list of float): List of fitness values for the population.

        Returns:
        - list of float: Cumulative fitness map.
        """
        fitmap = []
        total = 0
        for f in fits:
            total = total + f
            fitmap.append(total)
        return fitmap
    
    @staticmethod
    def select_parent(fitmap):
        """
        Selects a parent index based on the cumulative fitness map using stochastic universal sampling (SUS).

        Args:
        - fitmap (list of float): Cumulative fitness map.

        Returns:
        - int: Index of the selected parent in the population.
        """
        r = np.random.rand()  # Random number between 0 and 1
        r = r * fitmap[-1]  # Scale r to the total fitness
        for i in range(len(fitmap)):
            if r <= fitmap[i]:
                return i
