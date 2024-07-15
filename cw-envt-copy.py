"""
 cw-envt_copy.py

This script is designed to integrate PyBullet physics simulation with genetic algorithm-controlled creatures 
navigating through a dynamically generated environment, providing insights into their behavior and performance metrics. 
Adjustments can be made to the terrain, obstacles, and creature parameters to explore different simulation scenarios.
 
"""

import os 
import genome
import sys
import creature
import pybullet as p
import time 
import random
import numpy as np
import pybullet_data
import math
import glob
import matplotlib.pyplot as plt

# Initialize PyBullet in GUI mode and set the search path for additional data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Function to create an arena with walls and floor
def make_arena(arena_size=10, wall_height=1):
    wall_thickness = 0.5
    # Create the floor of the arena
    floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size / 2, arena_size / 2, wall_thickness])
    floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size / 2, arena_size / 2, wall_thickness], rgbaColor=[1, 1, 0, 1])
    floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

    # Create the north and south walls
    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size / 2, wall_height / 2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size / 2, wall_height / 2])

    # Create the east and west walls
    wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2])
    wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size / 2, 0, wall_height / 2])
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size / 2, 0, wall_height / 2])

# Function to create a mountainous terrain with rocks
def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
    def gaussian(x, y, sigma=arena_size / 4):
        """Return the height of the mountain at position (x, y) using a Gaussian function."""
        return mountain_height * math.exp(-((x ** 2 + y ** 2) / (2 * sigma ** 2)))

    for _ in range(num_rocks):
        # Randomly position rocks within the arena
        x = random.uniform(-1 * arena_size / 2, arena_size / 2)
        y = random.uniform(-1 * arena_size / 2, arena_size / 2)
        z = gaussian(x, y)  # Height determined by the Gaussian function

        # Adjust the size of the rocks based on height (higher rocks are smaller)
        size_factor = 1 - (z / mountain_height)
        size = random.uniform(0.1, max_size) * size_factor

        # Set random orientation for each rock
        orientation = p.getQuaternionFromEuler([random.uniform(0, math.pi), random.uniform(0, math.pi), random.uniform(0, math.pi)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)

# Function to create random rocks in the arena
def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
    for _ in range(num_rocks):
        x = random.uniform(-1 * arena_size/2, arena_size/2)
        y = random.uniform(-1 * arena_size/2, arena_size/2)
        z = 0.5  # Adjust based on your needs
        size = random.uniform(0.1,max_size)
        orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
        rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
        rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
        rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)

# Main function to simulate the environment and control the creature
def main(csv_file, connection_mode):
    """
    Main function to simulate a PyBullet environment where a creature controlled by a genetic algorithm
    navigates through obstacles and terrain. The creature's behavior is determined by genetic parameters
    loaded from a CSV file.

    Returns:
    - dist_moved: Total vertical distance moved by the creature during simulation.
    """
    assert os.path.exists(csv_file), "Tried to load " + csv_file + " but it does not exist"

    # PyBullet setup
    physicsClient = p.connect(connection_mode)
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    plane_shape = p.createCollisionShape(p.GEOM_PLANE)
    p.setGravity(0, 0, -10)

    # Create the arena and obstacles
    arena_size = 20
    make_arena(arena_size=arena_size)
    # Uncomment to add rocks or mountainous terrain'
    make_rocks(arena_size=arena_size)
    make_mountain(arena_size=arena_size)

    # Load the mountainous terrain from URDF files
    mountain_position = (0, 0, -1)
    mountain_position_2 = (10, -10, 1)
    mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
    p.setAdditionalSearchPath('shapes/')
    p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)
    # p.loadURDF("mountain_with_cubes.urdf", mountain_position_2, mountain_orientation, useFixedBase=1)

    # Create a creature using genetic parameters from CSV
    cr = creature.Creature(gene_count=5)
    print("Loading DNA from CSV file")
    dna = genome.Genome.from_csv(csv_file)
    print("DNA Loaded: ", dna)
    cr.update_dna(dna)
    print("Creature DNA updated")

    # Save creature to XML
    with open('test.urdf', 'w') as f:
        f.write(cr.to_xml())

    # Load the creature into the simulation
    rob1 = p.loadURDF('test.urdf', (0, 0, 10))

    # Initialize simulation parameters
    start_pos, orn = p.getBasePositionAndOrientation(rob1)
    elapsed_time = 0
    wait_time = 1.0 / 240
    total_time = 30
    step = 0
    dist_moved = 0

    # Simulation loop
    while True:
        p.stepSimulation()
        step += 1
        if step % 24 == 0:
            motors = cr.get_motors()
            assert len(motors) == p.getNumJoints(rob1), "Something went wrong"
            for jid in range(p.getNumJoints(rob1)):
                mode = p.VELOCITY_CONTROL
                vel = motors[jid].get_output()
                p.setJointMotorControl2(rob1, jid, controlMode=mode, targetVelocity=vel)
            new_pos, orn = p.getBasePositionAndOrientation(rob1)
            dist_moved = np.linalg.norm(np.asarray(start_pos) - np.asarray(new_pos))
            p.setJointMotorControl2(rob1, jid, controlMode=mode, targetVelocity=vel)
            print("new pos", new_pos)
            print("dist moved", dist_moved)
        time.sleep(wait_time)
        elapsed_time += wait_time
        if elapsed_time > total_time:
            break

    print("TOTAL DISTANCE MOVED:", dist_moved)
    # p.disconnect()  # Uncomment to disconnect from PyBullet simulation
    return dist_moved

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python cw-envt_copy.py csv_filename")
        sys.exit(1)
    
    csv_filename = sys.argv[1]
    main("ga_output.csv", p.DIRECT)  # Adjust as per your script's requirements


# """
#  cw-envt_copy.py

# This script is designed to integrate PyBullet physics simulation with genetic algorithm-controlled creatures 
# navigating through a dynamically generated environment, providing insights into their behavior and performance metrics. 
# Adjustments can be made to the terrain, obstacles, and creature parameters to explore different simulation scenarios.
 
# """

# import os 
# import genome
# import sys
# import creature
# import pybullet as p
# import time 
# import random
# import numpy as np
# import pybullet_data
# import math
# import glob
# import matplotlib.pyplot as plt

# # Initialize PyBullet in GUI mode and set the search path for additional data
# p.connect(p.GUI)
# p.setAdditionalSearchPath(pybullet_data.getDataPath())

# # Function to create an arena with walls and floor
# def make_arena(arena_size=10, wall_height=1):
#     wall_thickness = 0.5
#     # Create the floor of the arena
#     floor_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size / 2, arena_size / 2, wall_thickness])
#     floor_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size / 2, arena_size / 2, wall_thickness], rgbaColor=[1, 1, 0, 1])
#     floor_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=floor_collision_shape, baseVisualShapeIndex=floor_visual_shape, basePosition=[0, 0, -wall_thickness])

#     # Create the north and south walls
#     wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2])
#     wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[arena_size / 2, wall_thickness / 2, wall_height / 2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

#     p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, arena_size / 2, wall_height / 2])
#     p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[0, -arena_size / 2, wall_height / 2])

#     # Create the east and west walls
#     wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2])
#     wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[wall_thickness / 2, arena_size / 2, wall_height / 2], rgbaColor=[0.7, 0.7, 0.7, 1])  # Gray walls

#     p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[arena_size / 2, 0, wall_height / 2])
#     p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, baseVisualShapeIndex=wall_visual_shape, basePosition=[-arena_size / 2, 0, wall_height / 2])

# # Function to create a mountainous terrain with rocks
# def make_mountain(num_rocks=100, max_size=0.25, arena_size=10, mountain_height=5):
#     def gaussian(x, y, sigma=arena_size / 4):
#         """Return the height of the mountain at position (x, y) using a Gaussian function."""
#         return mountain_height * math.exp(-((x ** 2 + y ** 2) / (2 * sigma ** 2)))

#     for _ in range(num_rocks):
#         # Randomly position rocks within the arena
#         x = random.uniform(-1 * arena_size / 2, arena_size / 2)
#         y = random.uniform(-1 * arena_size / 2, arena_size / 2)
#         z = gaussian(x, y)  # Height determined by the Gaussian function

#         # Adjust the size of the rocks based on height (higher rocks are smaller)
#         size_factor = 1 - (z / mountain_height)
#         size = random.uniform(0.1, max_size) * size_factor

#         # Set random orientation for each rock
#         orientation = p.getQuaternionFromEuler([random.uniform(0, math.pi), random.uniform(0, math.pi), random.uniform(0, math.pi)])
#         rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
#         rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
#         rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)

# # Function to create random rocks in the arena
# def make_rocks(num_rocks=100, max_size=0.25, arena_size=10):
#     for _ in range(num_rocks):
#         x = random.uniform(-1 * arena_size/2, arena_size/2)
#         y = random.uniform(-1 * arena_size/2, arena_size/2)
#         z = 0.5  # Adjust based on your needs
#         size = random.uniform(0.1,max_size)
#         orientation = p.getQuaternionFromEuler([random.uniform(0, 3.14), random.uniform(0, 3.14), random.uniform(0, 3.14)])
#         rock_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
#         rock_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[0.5, 0.5, 0.5, 1])
#         rock_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rock_shape, baseVisualShapeIndex=rock_visual, basePosition=[x, y, z], baseOrientation=orientation)

# # Main function to simulate the environment and control the creature
# def main(csv_file, connection_mode):
#     """
#     Main function to simulate a PyBullet environment where a creature controlled by a genetic algorithm
#     navigates through obstacles and terrain. The creature's behavior is determined by genetic parameters
#     loaded from a CSV file.

#     Returns:
#     - dist_moved: Total vertical distance moved by the creature during simulation.
#     """
#     assert os.path.exists(csv_file), "Tried to load " + csv_file + " but it does not exist"

#     # PyBullet setup
#     physicsClient = p.connect(connection_mode)
#     p.setPhysicsEngineParameter(enableFileCaching=0)
#     p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
#     plane_shape = p.createCollisionShape(p.GEOM_PLANE)
#     p.setGravity(0, 0, -10)

#     # Parameter Tuning: Experiment with different parameters
#     arena_size = 20
#     num_rocks_list = [50, 100, 150]  # Different numbers of rocks to test
#     mountain_height_list = [3, 5, 7]  # Different mountain heights to test

#     results = {}

#     for num_rocks in num_rocks_list:
#         for mountain_height in mountain_height_list:
#             print(f"Testing with num_rocks={num_rocks}, mountain_height={mountain_height}")
            
#             # Create the arena and obstacles
#             make_arena(arena_size=arena_size)
#             make_rocks(num_rocks=num_rocks, arena_size=arena_size)
#             make_mountain(arena_size=arena_size, mountain_height=mountain_height)

#     # Create the arena and obstacles
#     # arena_size = 20
#     # make_arena(arena_size=arena_size)
#     # Uncomment to add rocks or mountainous terrain'
#     # make_rocks(arena_size=arena_size)
#     # make_mountain(arena_size=arena_size)

#     # Load the mountainous terrain from URDF files
#     mountain_position = (0, 0, -1)
#     mountain_position_2 = (10, -10, 1)
#     mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
#     p.setAdditionalSearchPath('shapes/')
#     p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)
#     # p.loadURDF("mountain_with_cubes.urdf", mountain_position_2, mountain_orientation, useFixedBase=1)

#     # Create a creature using genetic parameters from CSV
#     cr = creature.Creature(gene_count=5)
#     print("Loading DNA from CSV file")
#     dna = genome.Genome.from_csv(csv_file)
#     print("DNA Loaded: ", dna)
#     cr.update_dna(dna)
#     print("Creature DNA updated")

#     # Save creature to XML
#     with open('test.urdf', 'w') as f:
#         f.write(cr.to_xml())

#     # Load the creature into the simulation
#     rob1 = p.loadURDF('test.urdf', (0, 0, 10))

#     # Initialize simulation parameters
#     start_pos, orn = p.getBasePositionAndOrientation(rob1)
#     elapsed_time = 0
#     wait_time = 1.0 / 240
#     total_time = 30
#     step = 0
#     dist_moved = 0

#     # Simulation loop
#     while True:
#         p.stepSimulation()
#         step += 1
#         if step % 24 == 0:
#             motors = cr.get_motors()
#             assert len(motors) == p.getNumJoints(rob1), "Something went wrong"
#             for jid in range(p.getNumJoints(rob1)):
#                 mode = p.VELOCITY_CONTROL
#                 vel = motors[jid].get_output()
#                 p.setJointMotorControl2(rob1, jid, controlMode=mode, targetVelocity=vel)
#             new_pos, orn = p.getBasePositionAndOrientation(rob1)
#             dist_moved = np.linalg.norm(np.asarray(start_pos) - np.asarray(new_pos))
#             p.setJointMotorControl2(rob1, jid, controlMode=mode, targetVelocity=vel)
#             print("new pos", new_pos)
#             print("dist moved", dist_moved)
#         time.sleep(wait_time)
#         elapsed_time += wait_time
#         if elapsed_time > total_time:
#             break

#     print("TOTAL DISTANCE MOVED:", dist_moved)
#     # p.disconnect()  # Uncomment to disconnect from PyBullet simulation
#     return dist_moved

# if __name__ == "__main__":
#     if len(sys.argv) != 2:
#         print("Usage: python cw-envt_copy.py csv_filename")
#         sys.exit(1)
    
#     csv_filename = sys.argv[1]
#     main("ga_output.csv", p.DIRECT)  # Adjust as per your script's requirements
