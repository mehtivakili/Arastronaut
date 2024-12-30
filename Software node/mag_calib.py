import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def remove_outliers(data, threshold=1):
    """
    Remove outliers from data using a robust method across all axes.
    Any point with a combined Z-score higher than the threshold is considered an outlier.
    """
    # Calculate the mean and standard deviation for each axis
    mean = np.mean(data, axis=0)
    std_dev = np.std(data, axis=0)
    
    # Calculate Z-scores for the entire dataset
    z_scores = np.abs((data - mean) / std_dev)
    
    # Filter out any points where the Z-score exceeds the threshold on any axis
    filtered_data = data[(z_scores < threshold).all(axis=1)]
    return filtered_data

# Load data from the CSV file
file_path = 'E:/aras/arastronaut/arastronaut/Arastronaut/mag.csv'
data = np.loadtxt(file_path)

# Remove outliers
data_filtered = remove_outliers(data, threshold=2)

# Separate filtered data into X, Y, Z components
x_raw, y_raw, z_raw = data_filtered[:, 0], data_filtered[:, 1], data_filtered[:, 2]

# Offset correction: Centering the data around zero
offset_x = np.mean(x_raw)
offset_y = np.mean(y_raw)
offset_z = np.mean(z_raw)

x_corrected = x_raw - offset_x
y_corrected = y_raw - offset_y
z_corrected = z_raw - offset_z

# # Plot the raw data (after outlier removal but before offset correction)
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter(x_raw, y_raw, z_raw, c='r', marker='o', label='Filtered Raw Data')
# ax.set_xlabel('X Axis')
# ax.set_ylabel('Y Axis')
# ax.set_zlabel('Z Axis')
# ax.legend()
# plt.title('Filtered Raw Magnetometer Data')
# plt.show()

# Plot the corrected data (centered at zero)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x_raw, y_raw, z_raw, c='r', marker='o', label='Filtered Raw Data')
ax.scatter(x_corrected, y_corrected, z_corrected, c='g', marker='^', label='Corrected Data')
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.legend()
plt.title('Magnetometer Data - Centered at Zero')
plt.show()

# Output the raw and corrected data for inspection
print("Filtered Raw Data (X, Y, Z):")
for i in range(len(x_raw)):
    print(f"({x_raw[i]:.2f}, {y_raw[i]::.2f}, {z_raw[i]:.2f})")

print("\nCorrected Data (X, Y, Z):")
for i in range(len(x_corrected)):
    print(f"({x_corrected[i]:.2f}, {y_corrected[i]:.2f}, {z_corrected[i]:.2f})")
