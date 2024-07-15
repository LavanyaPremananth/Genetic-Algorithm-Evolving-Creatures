"""
test_simulation.py
"""

import unittest
import simulation
import creature
import population

class TestSim(unittest.TestCase):
    def testSimExists(self):
        """
        Test if an instance of Simulation exists.
        """
        sim = simulation.Simulation()
        self.assertIsNotNone(sim)

    def testSimId(self):
        """
        Test if Simulation has a valid physics client ID.
        """
        sim = simulation.Simulation()
        self.assertIsNotNone(sim.physicsClientId)

    def testRun(self):
        """
        Test if Simulation has a 'run_creature' method.
        """
        sim = simulation.Simulation()
        self.assertIsNotNone(sim.run_creature)

    def testPos(self):
        """
        Test if running a creature updates its position.
        """
        sim = simulation.Simulation()
        cr = creature.Creature(gene_count=3)
        sim.run_creature(cr)
        self.assertNotEqual(cr.start_position, cr.last_position)
    
    def testPop(self):
        """
        Test running simulation for each creature in a population.
        """
        pop = population.Population(pop_size=5, gene_count=3)
        sim = simulation.Simulation()
        for cr in pop.creatures:
            sim.run_creature(cr)
        dists = [cr.get_distance_travelled() for cr in pop.creatures]
        print(dists)
        self.assertIsNotNone(dists)

    # Uncomment the following method to test multi-threaded simulation
    
    # def testProc(self):
    #     """
    #     Test evaluation of population using multi-threaded simulation.
    #     """
    #     pop = population.Population(pop_size=20, gene_count=3)
    #     tsim = simulation.ThreadedSim(pool_size=8)
    #     tsim.eval_population(pop, 2400)
    #     dists = [cr.get_distance_travelled() for cr in pop.creatures]
    #     print(dists)
    #     self.assertIsNotNone(dists)

    def testProcNoThread(self):
        """
        Test evaluation of population using single-threaded simulation.
        """
        pop = population.Population(pop_size=20, gene_count=3)
        sim = simulation.Simulation()
        sim.eval_population(pop, 2400)
        dists = [cr.get_distance_travelled() for cr in pop.creatures]
        print(dists)
        self.assertIsNotNone(dists)
        
    def testPopSize5(self):
        """
        Test simulation with population size 5.
        """
        pop_size = 5
        pop = population.Population(pop_size=pop_size, gene_count=3)
        sim = simulation.Simulation()
        for cr in pop.creatures:
            sim.run_creature(cr)
        dists = [cr.get_distance_travelled() for cr in pop.creatures]
        self.assertEqual(len(dists), pop_size)
        self.assertIsNotNone(dists)

    def testPopSize10(self):
        """
        Test simulation with population size 10.
        """
        pop_size = 10
        pop = population.Population(pop_size=pop_size, gene_count=3)
        sim = simulation.Simulation()
        for cr in pop.creatures:
            sim.run_creature(cr)
        dists = [cr.get_distance_travelled() for cr in pop.creatures]
        self.assertEqual(len(dists), pop_size)
        self.assertIsNotNone(dists)

    def TestVerticalDistance (self):
        """
        Test vertical Distance.
        """
        sim = simulation.Simulation()
        cr = creature.Creature(gene_count = 3)
        sim.run_creature(cr)
        self.assertNotEqual(cr.start_position[2], cr.last_position[2])

unittest.main()
