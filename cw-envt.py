"""
cw-envt.py

This script sets up a PyBullet simulation environment for testing creatures evolved using a genetic algorithm.
It includes the creation of an arena with walls and a mountain using Gaussian distribution for the creatures to climb.
The script also loads a creature from a genetic algorithm into the simulation and runs the simulation in real-time.
"""

import pybullet as p
import pybullet_data
import time
import numpy as np
import random
import creature
import math

# Initialize PyBullet in GUI mode and set the search path for additional data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())


# Adjust camera distance to zoom out
camera_distance = 25.0  # Adjust this value as needed
camera_yaw = 0
camera_pitch = -30
p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, [0, 0, 0])

# Function to create a mountain using a Gaussian distribution
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

# Function to create rocks
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

# Function to create an arena with walls
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

# Set gravity for the simulation
p.setGravity(0, 0, -10)

# Create the arena and mountain
arena_size = 20
make_arena(arena_size=arena_size)
mountain_position = (0, 0, -1)  # Position of the mountain (adjust as needed)
mountain_orientation = p.getQuaternionFromEuler((0, 0, 0))
p.setAdditionalSearchPath('shapes/')
mountain = p.loadURDF("gaussian_pyramid.urdf", mountain_position, mountain_orientation, useFixedBase=1)

# Load and control the creature
cr = creature.Creature(gene_count=10)  # Initialize creature with a specified gene count

# Save creature to XML
with open('test.urdf', 'w') as f:
    f.write(cr.to_xml())


# Load the creature into the simulation
rob1 = p.loadURDF('test.urdf', (0, 0, 10))

# Define movement parameters
max_force = 100  # Maximum force applied to joints
joint_indices = []  # Fill this with the joint indices of your creature

# Example: Get joint indices from the loaded URDF robot
num_joints = p.getNumJoints(rob1)
for i in range(num_joints):
    joint_info = p.getJointInfo(rob1, i)
    joint_indices.append(joint_info[0])

# Enable torque control for each joint
for joint_index in joint_indices:
    p.setJointMotorControl2(rob1, joint_index, p.VELOCITY_CONTROL, force=max_force)

# Main simulation loop
p.setRealTimeSimulation(1)
while True:
    # Example: Move joints continuously (e.g., sinusoidal movement)
    for joint_index in joint_indices:
        target_velocity = math.sin(time.time())  # Example sinusoidal movement
        p.setJointMotorControl2(rob1, joint_index, p.VELOCITY_CONTROL, targetVelocity=target_velocity, force=max_force)
    
    time.sleep(0.01)  # Adjust time step as necessary for your simulation

# Enable real-time simulation and start the simulation loop
p.setRealTimeSimulation(1)
while True:
    time.sleep(0.1)  # Adjust time step as necessary for your simulation

