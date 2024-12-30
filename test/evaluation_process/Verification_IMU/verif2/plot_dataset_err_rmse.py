import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# File paths (modify these if your files have different names or locations)
encoder_file = 'dataset-encoder2.csv'
screw_axis_file = 'dataset-screw2.csv'

# Output file paths
error_output_file = 'absolute_error.csv'
rmse_output_file = 'rmse.txt'
plot_output_file = 'plots.png'  # Optional: Save the plots as an image

# ----------------------------- Step 1: Read the CSV Data ----------------------------- #

# Read the encoder data
encoder_df = pd.read_csv(encoder_file)

# Read the screw axis data
screw_axis_df = pd.read_csv(screw_axis_file)

# ---------------------- Step 2: Data Type Conversion and Cleaning ---------------------- #

# Convert timestamp_ns to integer (if not already)
encoder_df['timestamp_ns'] = encoder_df['timestamp_ns'].astype(int)
screw_axis_df['timestamp_ns'] = screw_axis_df['timestamp_ns'].astype(int)

# Ensure all angle values are positive by taking their absolute values
encoder_df['angle_deg'] = encoder_df['angle_deg'].abs()
screw_axis_df['angle_axis'] = screw_axis_df['angle_axis'].abs()

# Optional: Check for missing values and handle them
# print("Encoder Data Missing Values:\n", encoder_df.isnull().sum())
# print("Screw Axis Data Missing Values:\n", screw_axis_df.isnull().sum())
# encoder_df = encoder_df.dropna()
# screw_axis_df = screw_axis_df.dropna()

# ----------------------- Step 3: Aggregate Data by Timestamp ----------------------- #

# Process encoder data: If multiple entries have the same timestamp, take the average angle_deg
encoder_agg = encoder_df.groupby('timestamp_ns').agg({'angle_deg': 'mean'}).reset_index()

# Process screw axis data: If multiple entries have the same timestamp, take the average angle_axis
screw_axis_agg = screw_axis_df.groupby('timestamp_ns').agg({'angle_axis': 'mean'}).reset_index()

# ----------------------------- Step 4: Merge Datasets ----------------------------- #

# Merge the two datasets on timestamp_ns
merged_df = pd.merge(encoder_agg, screw_axis_agg, on='timestamp_ns', how='inner')

# ----------------------------- Step 5: Calculate Errors ----------------------------- #

# Calculate absolute error between encoder and screw axis angles
merged_df['absolute_error'] = (merged_df['angle_deg'] - merged_df['angle_axis']).abs()

# Convert timestamp from nanoseconds to seconds for plotting
merged_df['time_sec'] = merged_df['timestamp_ns'] / 1e9

# Sort by time to ensure the plots are ordered
merged_df = merged_df.sort_values('time_sec')

# ----------------------- Step 6: Save Absolute Error Data ----------------------- #

# Save the absolute error data to a CSV file
merged_df[['timestamp_ns', 'time_sec', 'absolute_error']].to_csv(error_output_file, index=False)
print(f"✅ Absolute error data saved to '{error_output_file}'")

# ------------------------- Step 7: Calculate and Save RMSE ------------------------- #

# Calculate RMSE
rmse = np.sqrt(np.mean(merged_df['absolute_error'] ** 2))
print(f"✅ Root Mean Squared Error (RMSE): {rmse:.4f}")

# Save the RMSE to a text file
with open(rmse_output_file, 'w') as f:
    f.write(f"Root Mean Squared Error (RMSE): {rmse:.4f}\n")
print(f"✅ RMSE value saved to '{rmse_output_file}'")

# ------------------------------- Step 8: Plotting ------------------------------- #

# Create a figure with three subplots
fig, axs = plt.subplots(3, 1, figsize=(14, 18), sharex=True)

# Plot 1: Absolute Encoder Degrees vs. Time
axs[0].plot(merged_df['time_sec'], merged_df['angle_deg'], marker='o', linestyle='-', color='blue')
axs[0].set_title('Absolute Encoder Degrees Over Time')
axs[0].set_ylabel('Encoder Angle (degrees)')
axs[0].grid(True)

# Plot 2: Absolute Screw Axis Degrees vs. Time
axs[1].plot(merged_df['time_sec'], merged_df['angle_axis'], marker='s', linestyle='-', color='green')
axs[1].set_title('Absolute Screw Axis Degrees Over Time')
axs[1].set_ylabel('Screw Axis Angle (degrees)')
axs[1].grid(True)

# Plot 3: Absolute Error vs. Time
axs[2].plot(merged_df['time_sec'], merged_df['absolute_error'], marker='^', linestyle='-', color='red')
axs[2].set_title('Absolute Error Between Encoder and Screw Axis Angles Over Time')
axs[2].set_xlabel('Time (seconds)')
axs[2].set_ylabel('Absolute Error (degrees)')
axs[2].grid(True)

# Adjust layout for better spacing
plt.tight_layout()

# Optional: Save the plots as an image
plt.savefig(plot_output_file)
print(f"✅ Plots saved as '{plot_output_file}'")

# Display the plots
plt.show()
