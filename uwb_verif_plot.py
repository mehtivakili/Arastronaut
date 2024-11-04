import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
data = pd.read_csv('time_distance_data.csv', header=None)

# Assign columns to variables
x_values = data[0]  # First column (X values)
y_values = data[1]  # Second column (Y values)

# Create the plot
plt.figure(figsize=(8, 6))
plt.plot(x_values, y_values, linestyle='-', color='b')

# Add labels and title
plt.xlabel('X Values')
plt.ylabel('Y Values')
plt.title('Time vs Distance Data Plot')

# Show grid and plot
plt.grid(True)
plt.show()
