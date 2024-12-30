import pandas as pd
import matplotlib.pyplot as plt

# File paths (modify these if your files have different names or locations)
encoder_file = 'dataset-encoder1.csv'
screw_axis_file = 'dataset-screw1.csv'

# Read the encoder data
encoder_df = pd.read_csv(encoder_file)

# Read the screw axis data
screw_axis_df = pd.read_csv(screw_axis_file)

# Convert timestamp_ns to integer (if not already)
encoder_df['timestamp_ns'] = encoder_df['timestamp_ns'].astype(int)
screw_axis_df['timestamp_ns'] = screw_axis_df['timestamp_ns'].astype(int)

# Ensure all angle values are positive by taking their absolute values
encoder_df['angle_deg'] = encoder_df['angle_deg'].abs()
screw_axis_df['angle_axis'] = screw_axis_df['angle_axis'].abs()

# Process encoder data: If multiple entries have the same timestamp, take the average angle_deg
encoder_agg = encoder_df.groupby('timestamp_ns').agg({'angle_deg': 'mean'}).reset_index()

# Process screw axis data: If multiple entries have the same timestamp, take the average angle_axis
screw_axis_agg = screw_axis_df.groupby('timestamp_ns').agg({'angle_axis': 'mean'}).reset_index()

# Merge the two datasets on timestamp_ns
merged_df = pd.merge(encoder_agg, screw_axis_agg, on='timestamp_ns', how='inner')

# Calculate absolute error between encoder and screw axis angles
merged_df['absolute_error'] = (merged_df['angle_deg'] - merged_df['angle_axis']).abs()

# Convert timestamp from nanoseconds to seconds for plotting
merged_df['time_sec'] = merged_df['timestamp_ns'] / 1e9

# Sort by time to ensure the plot is ordered
merged_df = merged_df.sort_values('time_sec')

# Plot the absolute error over time
plt.figure(figsize=(12, 6))
plt.plot(merged_df['time_sec'], merged_df['absolute_error'], marker='o', linestyle='-')
plt.title('Absolute Error between Encoder and Screw Axis Angles Over Time')
plt.xlabel('Time (seconds)')
plt.ylabel('Absolute Error (degrees)')
plt.grid(True)
plt.tight_layout()
plt.show()
