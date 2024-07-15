"""
test_population.py
"""

# Import necessary libraries and modules
import unittest
import population 
import numpy as np

# Define a test case class inheriting from unittest.TestCase
class TestPop(unittest.TestCase):

    ## check for a parent id in the range 0-2
    def testSelPar(self):
        """
        Test selection of parent ID within the range of 0 to 2.
        """
        fits = [2.5, 1.2, 3.4]
        fitmap = population.Population.get_fitness_map(fits)
        pid = population.Population.select_parent(fitmap)
        self.assertLess(pid, 3)

    ## parent id should be 1 as the first fitness is zero
    ## second is 1000 and third is 0.1 , so second should 
    ## almost always be selected
    def testSelPar2(self):
        """
        Test selection of parent ID based on fitness values.
        """
        fits = [0, 1000, 0.1]
        fitmap = population.Population.get_fitness_map(fits)
        pid = population.Population.select_parent(fitmap)
        self.assertEqual(pid, 1)    

# Run the unit tests if this script is executed directly
if __name__ == '__main__':
    unittest.main()
