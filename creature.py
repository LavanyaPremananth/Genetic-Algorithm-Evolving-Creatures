"""
creature.py

This script defines the structure and behavior of a creature within a genetic algorithm framework. 
The Creature class encapsulates the DNA, morphology, and movement of a creature, while the Motor 
class simulates the behavior of actuators. The creatures are generated based on a genome and are 
capable of evolving over time to perform tasks such as climbing a mountain.
"""

import genome 
from xml.dom.minidom import getDOMImplementation
from enum import Enum
import numpy as np

# Enumeration for different types of motor control waveforms
class MotorType(Enum):
    PULSE = 1
    SINE = 2

# Class representing a motor/actuator in the creature
class Motor:
    def __init__(self, control_waveform, control_amp, control_freq):
        # Determine motor type based on the control waveform value
        if control_waveform <= 0.5:
            self.motor_type = MotorType.PULSE
        else:
            self.motor_type = MotorType.SINE
        self.amp = control_amp
        self.freq = control_freq
        self.phase = 0
    
    # Function to calculate the motor output based on its type
    def get_output(self):
        self.phase = (self.phase + self.freq) % (np.pi * 2)  # Update the phase based on frequency
        if self.motor_type == MotorType.PULSE:
            if self.phase < np.pi:
                output = 1
            else:
                output = -1
        elif self.motor_type == MotorType.SINE:
            output = np.sin(self.phase)  # Sine wave output
        return output 

# Class representing a creature with a genetic blueprint
class Creature:
    def __init__(self, gene_count):
        self.spec = genome.Genome.get_gene_spec()
        self.dna = genome.Genome.get_random_genome(len(self.spec), gene_count)
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None
        self.positions = []

    # Function to generate flat links from the genome if not already generated
    def get_flat_links(self):
        if self.flat_links == None:
            gdicts = genome.Genome.get_genome_dicts(self.dna, self.spec)
            self.flat_links = genome.Genome.genome_to_links(gdicts)
        return self.flat_links
    
    # Function to expand flat links to a full structure
    def get_expanded_links(self):
        self.get_flat_links()
        if self.exp_links is not None:
            return self.exp_links
        
        exp_links = [self.flat_links[0]]
        genome.Genome.expandLinks(self.flat_links[0], self.flat_links[0].name, self.flat_links, exp_links)
        self.exp_links = exp_links
        return self.exp_links

    # Function to generate an XML representation of the creature
    def to_xml(self):
        self.get_expanded_links()
        domimpl = getDOMImplementation()
        adom = domimpl.createDocument(None, "start", None)
        robot_tag = adom.createElement("robot")
        for link in self.exp_links:
            robot_tag.appendChild(link.to_link_element(adom))
        first = True
        for link in self.exp_links:
            if first:  # Skip the root node
                first = False
                continue
            robot_tag.appendChild(link.to_joint_element(adom))
        robot_tag.setAttribute("name", "pepe")  # Assign a name to the robot
        return '<?xml version="1.0"?>' + robot_tag.toprettyxml()

    # Function to retrieve and initialize motors for the creature
    def get_motors(self):
        self.get_expanded_links()
        if self.motors is None:
            motors = []
            for i in range(1, len(self.exp_links)):
                l = self.exp_links[i]
                m = Motor(l.control_waveform, l.control_amp, l.control_freq)
                motors.append(m)
            self.motors = motors 
        return self.motors 
    
    # Function to update the creature's position
    def update_position(self, pos):
        if self.start_position is None:
            self.start_position = pos
        else:
            self.last_position = pos
        self.positions.append(pos)

    # Function to calculate the distance traveled by the creature
    def get_distance_travelled(self):
        if self.start_position is None or self.last_position is None:
            return 0
        p1 = np.asarray(self.start_position)
        p2 = np.asarray(self.last_position)
        dist = np.linalg.norm(p1 - p2)  # Euclidean distance
        return dist 
    
     # Function to calculate the vertical distance between two positions
    def get_total_vertical_distance_travelled(self):
        total_vertical_distance = 0
        for i in range(1, len(self.positions)):
            p1 = np.asarray(self.positions[i-1])
            p2 = np.asarray(self.positions[i])
            vertical_distance = p2[2] - p1[2] 
            if vertical_distance > 0: 
                total_vertical_distance += vertical_distance
        return total_vertical_distance

    # Function to update the DNA of the creature and reset related properties
    def update_dna(self, dna):
        self.dna = dna
        self.flat_links = None
        self.exp_links = None
        self.motors = None
        self.start_position = None
        self.last_position = None

