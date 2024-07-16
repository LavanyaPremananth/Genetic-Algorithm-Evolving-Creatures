# Genetic-Algorithm-Evolving-Creatures

## Project Overview
This project involves adapting an existing evolutionary algorithm to enable creatures to evolve and accomplish the challenge of climbing a mountain. The task necessitates a shift in the creatures' goals and their integration into a newly designed environment tailored for mountain climbing. This README outlines the implementation steps, experiments conducted, and findings related to optimizing the evolutionary process within this framework.

## Table of Contents
- [Sandbox Arena and Mountain Environment](#sandbox-arena-and-mountain-environment)
- [Overview of Genetic Algorithm](#overview-of-genetic-algorithm)
- [Fitness Function and Creature Behavior in Simulation](#fitness-function-and-creature-behavior-in-simulation)
- [Fitness Evaluation](#fitness-evaluation)
- [Landscapes Generated](#landscapes-generated)
- [Findings and Data Analysis](#findings-and-data-analysis)
- [Experiments](#experiments)
- [Evaluation Results](#evaluation-results)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Sandbox Arena and Mountain Environment
The sandbox arena and mountain environment were adapted from pre-existing codebases to provide a realistic simulation setting. This environment incorporates diverse terrain and obstacles to simulate the challenges of mountain climbing, encouraging the evolution of adaptive behaviors among the creatures. Rocks of varying sizes are randomly positioned, with heights adjusted according to a Gaussian distribution that defines the mountain's contours.

## Overview of Genetic Algorithm
Creatures in the simulation are represented as genomes, encoding structural and behavioral attributes through sequences of genes. These genes determine joint types, lengths, and control mechanisms essential for navigating and scaling the mountainous terrain. Specifications for different gene types, such as link shape, length, radius, mass, joint types, origins, and control parameters, are defined through the `get_gene_spec` method.

The genetic algorithm framework includes methods for:
- **Crossover:** `crossover`
- **Point Mutation:** `point_mutate`
- **Shrink Mutation:** `shrink_mutate`
- **Grow Mutation:** `grow_mutate`

These operations modify the creatures' genomes, influencing their physical structure and behavior within the PyBullet simulation environment. As genomes evolve across generations, creatures exhibit varying movement patterns and interaction capabilities.

## Fitness Function and Creature Behavior in Simulation
The fitness function evaluates and guides the evolution of creatures. In the simulation, creatures engage in actions and behaviors, including movement and motor control using defined motors for each link. Motors, characterized by different types (pulse or sine wave) and control parameters (amplitude and frequency), dictate how creatures interact with their environment.

Position tracking enables continuous updates of the creature's position, which are used to calculate metrics such as total distance traveled and vertical distance climbed for fitness evaluation.

## Fitness Evaluation
Fitness evaluation employs a fitness-proportional selection technique, known as roulette wheel selection. This method selects creatures as parents for the next generation based on fitness levels, favoring higher-performing creatures. Elitism is incorporated by reserving a segment of top-performing creatures from each generation.

Fitness metrics include:
- Total vertical distance traveled
- Horizontal distance covered
- Efficiency in movement

These metrics contribute to the fitness score, guiding the genetic algorithm's selection process.

## Landscapes Generated
The script generates various 3D shapes and landscapes using functions like Gaussian distribution and Perlin noise. Examples include:
- **Gaussian Pyramid:** `generate_gaussian_pyramid4`
- **Noisy Landscape:** `generate_noisy_landscape`
- **Combined Shapes:** `combine_shapes`

These shapes are exported to OBJ files, suitable for visualization and further use in 3D modeling and simulation projects.

## Findings and Data Analysis
Files for data analysis include:
- `data_analysis_create_summary_csv.py`
- `data_analysis_table_summary.py`
- `data_ga_output_graph.py`
- `data_elite.py`

The `cw-envt_copy.py` script integrates custom terrain and the genetic algorithm into the simulation environment. Research investigates the impact of genetic algorithm settings on evolutionary processes, documenting performance across different parameters, mutation rates, and population sizes.

## Experiments
The study explores the effect of varying mutation rates and population sizes on the genetic algorithm's performance. Types of mutations studied include point, shrink, and grow mutations, with rates ranging from 0.01 to 0.25. Population sizes vary from 2 to 10, with a consistent mutation rate of 0.25.

## Evaluation Results
Evaluation results are visualized using Jupyter Notebook, with plots illustrating the influence of mutation rates and population sizes on mountain climbing performance.

## Installation
1. **Clone the repository:**
   ```sh
   git clone <repository-url>
   cd <repository-directory>
