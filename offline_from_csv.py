
"""

offline_from_csv.py

 This script conducts a PyBullet simulation to test a creature's behavior using genetic parameters loaded from a CSV file.
 It initializes a simulation environment with a ground plane, generates a random creature based on genetic data,
 loads the creature into the simulation, and then runs a simulation loop. During each simulation step, it controls
 the creature's joints based on genetic motor outputs and calculates the distance moved by the creature over time.
""" 
import os 
import genome
import sys
import creature
import pybullet as p
import time 
import random
import numpy as np

# Function to run the main simulation
def main(csv_file):
    # Check if the specified CSV file exists
    assert os.path.exists(csv_file), "Tried to load " + csv_file + " but it does not exist"

    # Connect to the PyBullet physics simulation (DIRECT mode for headless operation)
    p.connect(p.DIRECT)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    # Create a ground plane for the simulation
    plane_shape = p.createCollisionShape(p.GEOM_PLANE)
    floor = p.createMultiBody(plane_shape, plane_shape)

    # Set gravity for the simulation
    p.setGravity(0, 0, -10)
    # (Commenting out real-time simulation setting for now)
    # p.setRealTimeSimulation(1)

    # Generate a random creature with 1 gene (for simplicity in this example)
    cr = creature.Creature(gene_count=1)

    # Load genetic data from the CSV file and update the creature's DNA
    dna = genome.Genome.from_csv(csv_file)
    cr.update_dna(dna)

    # Save the creature as a URDF file for simulation
    with open('test.urdf', 'w') as f:
        f.write(cr.to_xml())

    # Load the creature into the simulation
    rob1 = p.loadURDF('test.urdf')

    # Position the creature in the air
    p.resetBasePositionAndOrientation(rob1, [0, 0, 2.5], [0, 0, 0, 1])

    # Get the initial position of the creature
    start_pos, orn = p.getBasePositionAndOrientation(rob1)

    # Simulation parameters
    elapsed_time = 0
    wait_time = 1.0 / 240  # Simulation time step in seconds
    total_time = 30  # Total simulation time in seconds
    step = 0

    # Simulation loop
    while True:
        p.stepSimulation()
        step += 1

        # Update motor controls for the creature every 1 second (24 simulation steps)
        if step % 24 == 0:
            motors = cr.get_motors()
            assert len(motors) == p.getNumJoints(rob1), "Number of motors doesn't match number of joints"

            # Set velocity control for each joint based on genetic motor outputs
            for jid in range(p.getNumJoints(rob1)):
                mode = p.VELOCITY_CONTROL
                vel = motors[jid].get_output()
                p.setJointMotorControl2(rob1, jid, controlMode=mode, targetVelocity=vel)

            # Calculate and print the distance moved by the creature
            new_pos, orn = p.getBasePositionAndOrientation(rob1)
            dist_moved = np.linalg.norm(np.asarray(start_pos) - np.asarray(new_pos))
            print("Distance moved:", dist_moved)

        # Uncomment the line below to simulate in real-time (commented for batch processing)
        # time.sleep(wait_time)

        elapsed_time += wait_time

        # Break out of loop after total_time seconds
        if elapsed_time > total_time:
            break

    # Print the total distance moved by the creature during the simulation
    print("TOTAL DISTANCE MOVED:", dist_moved)


if __name__ == "__main__":
    # Check if the script is run with the correct command line arguments
    assert len(sys.argv) == 2, "Usage: python playback_test.py csv_filename"
    main(sys.argv[1])
