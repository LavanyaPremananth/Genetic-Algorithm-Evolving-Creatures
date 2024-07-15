"""
motor_test.py

This script utilizes the PyBullet physics engine to simulate and manipulate 
a virtual creature defined by genetic parameters. 

Simulation Setup: Initializes a PyBullet simulation environment with a GUI, 
sets up a ground plane, and configures simulation parameters like gravity.

Creature Generation: Creates a random creature using genetic parameters 
defined in the creature module, then exports it to a URDF file (test.urdf) 
for simulation.

Simulation Loop: Steps through the simulation, updating the creature's 
movement and calculating the distance moved over time. It controls the 
creature's joints based on genetic motor outputs.

Output: Prints and finally logs the total distance moved by the creature 
during the simulation.
"""

import genome
import creature
import pybullet as p
import time 
import random
import numpy as np

# Initialize PyBullet simulation environment with GUI
p.connect(p.GUI)
p.setPhysicsEngineParameter(enableFileCaching=0)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Create a ground plane for the simulation
plane_shape = p.createCollisionShape(p.GEOM_PLANE)
floor = p.createMultiBody(plane_shape, plane_shape)

# Set gravity for the simulation
p.setGravity(0, 0, -10)
# (Commenting out real-time simulation setting for Mac compatibility)
# p.setRealTimeSimulation(1)

# Generate a random creature using 3 genes
cr = creature.Creature(gene_count=3)

# Save the creature as a URDF file for simulation
with open('test.urdf', 'w') as f:
    f.write(cr.to_xml())

# Load the creature into the simulation
rob1 = p.loadURDF('test.urdf')
start_pos, orn = p.getBasePositionAndOrientation(rob1)

# Simulation parameters
elapsed_time = 0
wait_time = 1.0 / 240  # Simulation time step in seconds
total_time = 5  # Total simulation time in seconds
step = 0
dist_moved = 0

# Simulation loop
while True:
    p.stepSimulation()
    step += 1

    # Update motor controls for the creature every 2 seconds (120 simulation steps)
    if step % 120 == 0:
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

    # Control simulation rate
    time.sleep(wait_time)
    elapsed_time += wait_time

    # Break out of loop after total_time seconds
    if elapsed_time > total_time:
        break

# Print the total distance moved by the creature during the simulation
print("TOTAL DISTANCE MOVED:", dist_moved)
