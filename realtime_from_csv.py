"""
realtime_from_csv.py

This script uses PyBullet to simulate the movement of a creature based on genetic data loaded from a CSV file. 
The creature's genetic makeup is represented using a genome, and its movement is controlled by joint velocities 
derived from genetic parameters. The simulation tracks the creature's movement over a specified time period, 
outputting the distance moved and total distance at the end. To provide a valid CSV file containing genetic 
data as a command-line argument when running the script.
"""

import os 
import genome
import sys
import creature
import pybullet as p
import time 
import random
import numpy as np

## ... usual starter code to create a sim and floor
def main(csv_file):
    assert os.path.exists(csv_file), "Tried to load " + csv_file + " but it does not exists"

    # Connect to the PyBullet GUI and configure simulation parameters
    p.connect(p.GUI)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Create a ground plane
    plane_shape = p.createCollisionShape(p.GEOM_PLANE)
    floor = p.createMultiBody(plane_shape, plane_shape)
    p.setGravity(0, 0, -10)
    
    # Generate a random creature and load genetic data
    cr = creature.Creature(gene_count=1)
    dna = genome.Genome.from_csv(csv_file)
    cr.update_dna(dna)
    
    # Save creature's XML representation to a file
    with open('test.urdf', 'w') as f:
        f.write(cr.to_xml())
    
    # Load creature into the simulation
    rob1 = p.loadURDF('test.urdf')
    
    # Position the creature above the ground plane
    p.resetBasePositionAndOrientation(rob1, [0, 0, 2.5], [0, 0, 0, 1])
    start_pos, orn = p.getBasePositionAndOrientation(rob1)

    # Simulation parameters
    elapsed_time = 0
    wait_time = 1.0 / 240  # seconds (simulation step time)
    total_time = 30  # seconds (total simulation time)
    step = 0

    # Simulation loop
    while True:
        p.stepSimulation()
        step += 1
        
        # Control joints every 1/10 seconds (24 steps)
        if step % 24 == 0:
            motors = cr.get_motors()
            assert len(motors) == p.getNumJoints(rob1), "Mismatch in number of joints"
            
            # Set joint velocities based on motor outputs
            for jid in range(p.getNumJoints(rob1)):
                mode = p.VELOCITY_CONTROL
                vel = motors[jid].get_output()
                p.setJointMotorControl2(rob1, jid, controlMode=mode, targetVelocity=vel)
            
            # Measure current position and calculate distance moved
            new_pos, orn = p.getBasePositionAndOrientation(rob1)
            dist_moved = np.linalg.norm(np.asarray(start_pos) - np.asarray(new_pos))
            print("Distance moved:", dist_moved)
        
        # Control simulation rate and elapsed time
        time.sleep(wait_time)
        elapsed_time += wait_time
        
        # End simulation after specified total time
        if elapsed_time > total_time:
            break
    
    # Print total distance moved by the creature
    print("TOTAL DISTANCE MOVED:", dist_moved)

if __name__ == "__main__":
    assert len(sys.argv) == 2, "Usage: python playback_test.py csv_filename"
    main(sys.argv[1])