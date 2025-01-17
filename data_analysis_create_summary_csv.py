import pandas as pd
import glob
import matplotlib.pyplot as plt
import os

data_folder = 'data'  # Folder where elite.csv files are stored

# Check if ga_output.csv exists
if not os.path.exists('ga_output.csv'):
    # If it doesn't exist, create a DataFrame with the necessary columns
    df = pd.DataFrame(columns=["iteration", "fittest", "mean"])
    # Save the DataFrame to ga_output.csv
    df.to_csv('ga_output.csv', index=False)

# Create an empty DataFrame to store all data
all_data = pd.DataFrame()

# Loop over all CSV files, sorted in ascending order by filename
for csv_file in sorted(glob.glob(os.path.join(data_folder, '*.csv'))):
    if csv_file == 'ga_output.csv':
        continue  # Ignore ga_output.csv file

    # Read the CSV file, skipping any whitespace after the comma delimiter
    df = pd.read_csv(csv_file, header=None, skipinitialspace=True)
    df = df.iloc[:, :-1]  # Drop the last column

    # Convert all values to numeric, replacing any non-numeric values with NaN
    df = df.apply(pd.to_numeric, errors='coerce')

    # Calculate some statistics
    fitness = df.sum(axis=1).mean()   
    num_links = df.count(axis=1).mean()  
    mean_vertical_distance = df[0].sum()   

    # Add the statistics to the DataFrame
    all_data = pd.concat([all_data, pd.DataFrame([{
        'file': csv_file,
        'fitness': fitness,
        'num_links': num_links,
        'vertical_distance': mean_vertical_distance
    }])], ignore_index=True)

# Save the DataFrame to a new CSV file
all_data.to_csv('summary.csv', index=False)

# Load the data from ga_output.csv
df_ga_output = pd.read_csv("ga_output.csv")

# Create a new figure for the first graph
plt.figure()

# Plot the maximum fitness at each iteration
plt.plot(df_ga_output["iteration"], df_ga_output["fittest"], label="Max Fitness")

# Plot the average fitness at each iteration
plt.plot(df_ga_output["iteration"], df_ga_output["mean"], label="Average Fitness")

# Plot the vertical distance at each generation
plt.plot(all_data.index, all_data['vertical_distance'], label="Vertical Distance")

# Add a legend
plt.legend()

# Add labels for the x and y axes
plt.xlabel("Generation")
plt.ylabel("Fitness")

# Create a new figure for the second graph
plt.figure(figsize=(10, 6))
plt.xlabel('Index')  # X-axis label
plt.ylabel('Value')  # Y-axis label
plt.title('Metrics Over Time For Each CSV file')  # Graph title

# Plot the fitness, number of links, and vertical distance
plt.plot(all_data['fitness'], label='Fitness')
plt.plot(all_data['num_links'], label='Number of Links')
plt.plot(all_data['vertical_distance'], label='Vertical Distance')

# Add a legend
plt.legend()

# Show the plot
plt.show()