"""
simulation.py

"""

import pybullet as p
from multiprocessing import Pool

class Simulation:
    def __init__(self, sim_id=0):
        """
        Initialize a simulation environment in PyBullet.

        Args:
        - sim_id (int): Simulation identifier.

        Initializes PyBullet in direct mode for physics simulation.
        """
        self.physicsClientId = p.connect(p.DIRECT)  # Connect to PyBullet in direct mode
        self.sim_id = sim_id  # Simulation ID

    def run_creature(self, cr, iterations=2400):
        """
        Run a creature simulation in PyBullet.

        Args:
        - cr (Creature): Creature object to simulate.
        - iterations (int): Number of simulation steps.

        Resets the simulation, loads the creature, and runs it for specified iterations.
        """
        pid = self.physicsClientId
        p.resetSimulation(physicsClientId=pid)  # Reset simulation
        p.setPhysicsEngineParameter(enableFileCaching=0, physicsClientId=pid)  # Configure physics parameters

        p.setGravity(0, 0, -10, physicsClientId=pid)  # Set gravity
        plane_shape = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=pid)  # Create ground plane
        floor = p.createMultiBody(plane_shape, plane_shape, physicsClientId=pid)

        # Define the position and orientation of the mountain
        mountain_position_2 = (10, -10, 1)  # Adjust as needed
        mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))

        # Set the additional search path for URDF files
        p.setAdditionalSearchPath('shapes/')

        # Load the URDF file for the mountain using the specified physics client ID
        landscape = p.loadURDF('mountain.urdf', useFixedBase=True, physicsClientId=pid)

        # Load another URDF file for a mountain with cubes at the specified position and orientation,
        # using a fixed base
        mountain = p.loadURDF("mountain_with_cubes.urdf", mountain_position_2, mountain_orientation, useFixedBase=1)

        xml_file = 'temp' + str(self.sim_id) + '.urdf'
        xml_str = cr.to_xml()
        with open(xml_file, 'w') as f:
            f.write(xml_str)
        
        cid = p.loadURDF(xml_file, physicsClientId=pid)  # Load creature URDF
        p.resetBasePositionAndOrientation(cid, [0, 0, 2.5], [0, 0, 0, 1], physicsClientId=pid)  # Position creature

        for step in range(iterations):
            p.stepSimulation(physicsClientId=pid)  # Step simulation
            if step % 24 == 0:
                self.update_motors(cid=cid, cr=cr)  # Update motors periodically

            pos, orn = p.getBasePositionAndOrientation(cid, physicsClientId=pid)
            cr.update_position(pos)  # Update creature's position

    def update_motors(self, cid, cr):
        """
        Update motors of the creature in the simulation.

        Args:
        - cid (int): Creature ID in the physics engine.
        - cr (Creature): Creature object with motor controls.

        Updates joint motor controls based on the creature's motor outputs.
        """
        for jid in range(p.getNumJoints(cid, physicsClientId=self.physicsClientId)):
            m = cr.get_motors()[jid]

            p.setJointMotorControl2(cid, jid,
                                    controlMode=p.VELOCITY_CONTROL,
                                    targetVelocity=m.get_output(),
                                    force=5,
                                    physicsClientId=self.physicsClientId)
        
    # Evaluate the population of creatures over a specified number of iterations.
    def eval_population(self, pop, iterations):
        for cr in pop.creatures:
            self.run_creature(cr, 2400) 

class ThreadedSim():
    def __init__(self, pool_size):
        """
        Initialize a pool of simulations for multi-threaded evaluation.

        Creates a list of Simulation instances to handle concurrent simulations.
        """
        self.sims = [Simulation(i) for i in range(pool_size)]

    @staticmethod
    def static_run_creature(sim, cr, iterations):
        """
        Static method to run a creature simulation in a specific simulation instance.

        Runs a creature simulation in a specified Simulation instance.
        """
        sim.run_creature(cr, iterations)
        return cr
    
    def eval_population(self, pop, iterations):
        """
        Evaluate a population of creatures across multiple simulations.

        Distributes the evaluation of creatures in the population across multiple simulations.
        """
        pool_args = []
        start_ind = 0
        pool_size = len(self.sims)
        
        while start_ind < len(pop.creatures):
            this_pool_args = []
            for i in range(start_ind, start_ind + pool_size):
                if i == len(pop.creatures):  # Check end of population
                    break
                sim_ind = i % len(self.sims)  # Determine simulation index
                this_pool_args.append([self.sims[sim_ind], pop.creatures[i], iterations])
            pool_args.append(this_pool_args)
            start_ind = start_ind + pool_size
        
        new_creatures = []
        for pool_argset in pool_args:
            with Pool(pool_size) as p:
                creatures = p.starmap(ThreadedSim.static_run_creature, pool_argset)
                new_creatures.extend(creatures)  # Collect results from simulations
        
        pop.creatures = new_creatures  # Update population with evaluated creatures
