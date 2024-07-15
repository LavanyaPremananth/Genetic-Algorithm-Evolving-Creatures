"""
The script is used to analyze and visualize data stored in summary.csv from a simulation or experiment. 
It provides statistical summaries and plots for fitness, number of links, and vertical distance metrics over time or iterations. 
These visualizations aid in understanding the performance and evolution of the simulated system.
"""

import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('summary.csv')

# Display basic statistics
print(df.describe())

# Plot fitness over time
plt.figure(figsize=(10, 6))
plt.plot(df['fitness'])
plt.title('Fitness over time')
plt.xlabel('CSV File')
plt.ylabel('Fitness')
plt.grid(True)
plt.show()

# Plot number of links over time
plt.figure(figsize=(10, 6))
plt.plot(df['num_links'])
plt.title('Number of links over time')
plt.xlabel('CSV File')
plt.ylabel('Number of links')
plt.grid(True)
plt.show()

# Plot vertical distance over time
plt.figure(figsize=(10, 6))
plt.plot(df['vertical_distance'])
plt.title('Vertical Distance over time')
plt.xlabel('CSV File')
plt.ylabel('Vertical Distance')
plt.grid(True)
plt.show()
