"""

test_genome.py 

 This script contains unit tests for the genome module using the unittest framework.
 It verifies various functionalities and methods of the Genome class, ensuring that they
 behave correctly under different conditions. 
 
 The purpose of these tests is to validate the reliability and correctness of 
 the genome operations implemented in the module.
 
"""

# Import necessary libraries and modules
import unittest 
import genome
import numpy as np
import os

# Define a test case class inheriting from unittest.TestCase
class GenomeTest(unittest.TestCase):

    def testClassExists(self):
        """
        Test if the Genome class exists.
        """
        self.assertIsNotNone(genome.Genome)

    def testRandomGene(self):
        """
        Test the existence of the method to generate random genes.
        """
        self.assertIsNotNone(genome.Genome.get_random_gene)

    def testRandomGeneNotNone(self):
        """
        Test that the generated random genes are not None.
        """
        self.assertIsNotNone(genome.Genome.get_random_gene(5))

    def testRandomGeneHasValues(self):
        """
        Test that random genes contain values.
        """
        gene = genome.Genome.get_random_gene(5)
        self.assertIsNotNone(gene[0])

    def testRandomGeneLength(self):
        """
        Test the length of randomly generated genes.
        """
        gene = genome.Genome.get_random_gene(20)
        self.assertEqual(len(gene), 20)

    def testRandGeneIsNumpyArrays(self):
        """
        Test that randomly generated genes are numpy arrays.
        """
        gene = genome.Genome.get_random_gene(20)
        self.assertEqual(type(gene), np.ndarray)

    def testRandomGenomeExists(self):
        """
        Test the existence of a randomly generated genome.
        """
        data = genome.Genome.get_random_genome(20, 5)
        self.assertIsNotNone(data)

    def testGeneSpecExist(self):
        """
        Test if gene specifications exist.
        """
        spec = genome.Genome.get_gene_spec()
        self.assertIsNotNone(spec)
    
    def testGeneSpecHasLinkLength(self):
        """
        Test that gene specifications include 'link-length'.
        """
        spec = genome.Genome.get_gene_spec()
        self.assertIsNotNone(spec['link-length'])

    def testGeneSpecHasLinkLengthInd(self):
        """
        Test that gene specifications include 'link-length' index.
        """
        spec = genome.Genome.get_gene_spec()
        self.assertIsNotNone(spec['link-length']["ind"])

    def testGeneSpecScale(self):
        """
        Test scaling of genes based on gene specifications.
        """
        spec = genome.Genome.get_gene_spec()
        gene = genome.Genome.get_random_gene(20)
        self.assertGreater(gene[spec["link-length"]["ind"]], 0)

    def testGeneToGeneDict(self):
        """
        Test conversion of genes to gene dictionaries.
        """
        spec = genome.Genome.get_gene_spec()
        gene = genome.Genome.get_random_gene(len(spec))
        gene_dict = genome.Genome.get_gene_dict(gene, spec)
        self.assertIn("link-recurrence", gene_dict)

    def testGenomeToDict(self):
        """
        Test conversion of genomes to dictionaries.
        """
        spec = genome.Genome.get_gene_spec()
        dna = genome.Genome.get_random_genome(len(spec), 3)
        genome_dicts = genome.Genome.get_genome_dicts(dna, spec)
        self.assertEqual(len(genome_dicts), 3)

    def testFlatLinks(self):
        """
        Test creation and existence of URDFLink objects.
        """
        links = [
          genome.URDFLink(name="A", parent_name=None, recur=1), 
          genome.URDFLink(name="B", parent_name="A", recur=2), 
          genome.URDFLink(name="C", parent_name="B", recur=2)
        ]
        self.assertIsNotNone(links)

    def testExpandLinks(self):
        """
        Test expansion of link objects.
        """
        links = [
            genome.URDFLink(name="A", parent_name="None", recur=1), 
            genome.URDFLink(name="B", parent_name="A", recur=1), 
            genome.URDFLink(name="C", parent_name="B", recur=2), 
            genome.URDFLink(name="D", parent_name="C", recur=1), 
        ]
        exp_links = [links[0]]
        genome.Genome.expandLinks(links[0], links[0].name, links, exp_links)   
        self.assertEqual(len(exp_links), 6)

    def testCrossover(self):
        """
        Test crossover operation between two genes.
        """
        g1 = [[1], [2], [3]]
        g2 = [[4], [5], [6]]
        for i in range(10):
            g3 = genome.Genome.crossover(g1, g2)
            self.assertGreater(len(g3), 0)
    
    def test_point(self):
        """
        Test point mutation on genes using numpy arrays.
        """
        g1 = np.array([[1.0], [2.0], [3.0]])
        g2 = genome.Genome.point_mutate(g1, rate=1, amount=0.25)
        self.assertFalse(np.array_equal(g1, g2))
    
    def test_point_range(self):
        """
        Test mutation range for genes using numpy arrays.
        """
        g1 = np.array([[1.0], [0.0], [1.0], [0.0]])
        for i in range(100):
            g2 = genome.Genome.point_mutate(g1, rate=1, amount=0.25)
            self.assertLessEqual(np.max(g2), 1.0)
            self.assertGreaterEqual(np.min(g2), 0.0)
    
    def test_shrink(self):
        """
        Test shrink mutation on genes.
        """
        g1 = np.array([[1.0], [2.0]])
        g2 = genome.Genome.shrink_mutate(g1, rate=1.0)
        # should def. shrink as rate = 1
        self.assertEqual(len(g2), 1) 

    def test_shrink2(self):
        """
        Test shrink mutation on genes when rate is 0.
        """
        g1 = np.array([[1.0], [2.0]])
        g2 = genome.Genome.shrink_mutate(g1, rate=0.0)
        # should not shrink as rate = 0
        self.assertEqual(len(g2), 2) 

    def test_shrink3(self):
        """
        Test no shrink mutation when gene length is 1.
        """
        g1 = np.array([[1.0]])
        g2 = genome.Genome.shrink_mutate(g1, rate=1.0)
        # should not shrink if already len 1
        self.assertEqual(len(g2), 1) 
    
    def test_grow1(self):
        """
        Test growth mutation on genes.
        """
        g1 = np.array([[1.0], [2.0]])
        g2 = genome.Genome.grow_mutate(g1, rate=1)
        self.assertGreater(len(g2), len(g1))

    def test_grow2(self):
        """
        Test no growth mutation on genes when rate is 0.
        """
        g1 = np.array([[1.0], [2.0]])
        g2 = genome.Genome.grow_mutate(g1, rate=0)
        self.assertEqual(len(g2), len(g1))

    def test_tocsv(self):
        """
        Test conversion of genes to CSV format and file creation.
        """
        g1 = [[1,2,3]]
        genome.Genome.to_csv(g1, 'test.csv')
        self.assertTrue(os.path.exists('test.csv'))

    def test_tocsv_content(self):
        """
        Test correctness of content when genes are converted to CSV format.
        """
        g1 = [[1,2,3]]
        genome.Genome.to_csv(g1, 'test.csv')
        expect = "1,2,3,\n"
        with open('test.csv') as f:
            csv_str = f.read() 
        self.assertEqual(csv_str, expect)

    def test_tocsv_content2(self):
        """
        Test correctness of content when multiple genes are converted to CSV format.
        """
        g1 = [[1,2,3], [4,5,6]]
        genome.Genome.to_csv(g1, 'test.csv')
        expect = "1,2,3,\n4,5,6,\n"
        with open('test.csv') as f:
            csv_str = f.read() 
        self.assertEqual(csv_str, expect)

    def test_from_csv(self):
        """
        Test reading genes from CSV and ensuring equality with original genes.
        """
        g1 = [[1,2,3]]
        genome.Genome.to_csv(g1, 'test.csv')
        g2 = genome.Genome.from_csv('test.csv')
        self.assertTrue(np.array_equal(g1, g2))

    def test_from_csv2(self):
        """
        Test reading multiple genes from CSV and ensuring equality with original genes.
        """
        g1 = [[1,2,3], [4,5,6]]
        genome.Genome.to_csv(g1, 'test.csv')
        g2 = genome.Genome.from_csv('test.csv')
        self.assertTrue(np.array_equal(g1, g2))

# Run the unit tests if this script is executed directly
if __name__ == '__main__':
    unittest.main()
