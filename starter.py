"""
starter.py
"""

import pybullet as p
import pybullet_data as pd

# Connect to PyBullet GUI
p.connect(p.GUI)

# Set physics engine parameters
p.setPhysicsEngineParameter(enableFileCaching=0)

# Configure debug visualizer
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# Create ground plane
plane_shape = p.createCollisionShape(p.GEOM_PLANE)
floor = p.createMultiBody(plane_shape, plane_shape)

# Set gravity
p.setGravity(0, 0, -10)
