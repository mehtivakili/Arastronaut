import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# -----------------------------
# 1. Read Data from CSV
# -----------------------------

csv_file_path = 'time_distance_data1.csv'

try:
    df = pd.read_csv(csv_file_path)
except FileNotFoundError:
    print(f"Error: The file '{csv_file_path}' was not found.")
    exit(1)
except pd.errors.EmptyDataError:
    print(f"Error: The file '{csv_file_path}' is empty.")
    exit(1)
except pd.errors.ParserError:
    print(f"Error: The file '{csv_file_path}' does not appear to be in CSV format.")
    exit(1)

# Verify that required columns exist
required_columns = ['Time (s)', 'Distance (m)']
if not all(col in df.columns for col in required_columns):
    print(f"Error: The CSV file must contain the following columns: {required_columns}")
    exit(1)

# Display the first few rows to verify
print("First few rows of the dataset:")
print(df.head())

# -----------------------------
# 2. Preprocess the Data
# -----------------------------

df = df.sort_values('Time (s)').reset_index(drop=True)
df = df[(df['Time (s)'] >= 0) & (df['Distance (m)'] >= 0)]
df = df.dropna(subset=['Time (s)', 'Distance (m)'])

# -----------------------------
# 3. Define 20-Second Intervals and 15-Second Middle Sub-Intervals
# -----------------------------

total_window_size = 20
sub_window_size = 15

max_time = df['Time (s)'].max()
num_windows = int(np.ceil(max_time / total_window_size))

window_start_times = []
sub_window_boundaries = []

for i in range(num_windows):
    window_start = i * total_window_size
    sub_window_start = window_start + (total_window_size - sub_window_size) / 2
    sub_window_end = sub_window_start + sub_window_size
    window_start_times.append(window_start)
    sub_window_boundaries.append((sub_window_start, sub_window_end))

print("\n20-Second Windows and Their 15-Second Middle Sub-Intervals:")
for idx, (start, (sub_start, sub_end)) in enumerate(zip(window_start_times, sub_window_boundaries)):
    print(f"Window {idx + 1}: {start} - {start + total_window_size} s | Sub-Window: {sub_start} - {sub_end} s")

# -----------------------------
# 4. Calculate Average Distance for Each 15-Second Sub-Interval
# -----------------------------

average_distances = []

for idx, (sub_start, sub_end) in enumerate(sub_window_boundaries):
    sub_window_data = df[(df['Time (s)'] >= sub_start) & (df['Time (s)'] < sub_end)]
    
    if not sub_window_data.empty:
        avg_distance = sub_window_data['Distance (m)'].mean()
    else:
        avg_distance = np.nan  # Assign NaN if no data points in the sub-window
    
    average_distances.append({
        'Window_Number': idx + 1,
        'Window_Start_Time': sub_start,
        'Window_End_Time': sub_end,
        'Average_Distance (m)': avg_distance
    })

average_distances_df = pd.DataFrame(average_distances)

print("\nAverage Distances for Each 15-Second Middle Sub-Interval:")
print(average_distances_df)

# -----------------------------
# 5. Generate Ground Truth Data
# -----------------------------

ground_truth_distances = []

for idx, row in average_distances_df.iterrows():
    window_end_time = row['Window_End_Time']
    if window_end_time <= 405:
        gt_distance = 0.41 * (row['Window_Number'] - 1 ) + 0.41
    else:
        gt_distance = 8.6
    ground_truth_distances.append(gt_distance)

average_distances_df['Ground_Truth_Distance (m)'] = ground_truth_distances

print("\nGround Truth Distances:")
print(average_distances_df[['Window_Number', 'Window_Start_Time', 'Ground_Truth_Distance (m)']])

# -----------------------------
# 6. Manually Compute RMSE between Measured and Ground Truth Distances
# -----------------------------

# Drop NaN values for RMSE calculation
valid_df = average_distances_df.dropna(subset=['Average_Distance (m)', 'Ground_Truth_Distance (m)'])

# Manually calculate RMSE
squared_errors = []
print("\nManual Calculation of Differences and Squared Errors:")
for index, row in valid_df.iterrows():
    measured = row['Average_Distance (m)']
    ground_truth = row['Ground_Truth_Distance (m)']
    
    # Calculate difference and squared error
    difference = measured - ground_truth
    squared_error = difference ** 2
    
    # Append to list
    squared_errors.append(squared_error)
    
    # Print the differences and squared errors
    print(f"Step {index+1}: Measured = {measured:.2f}, Ground Truth = {ground_truth:.2f}, Difference = {difference:.4f}, Squared Error = {squared_error:.4f}")

# Mean of squared errors
mean_squared_error_manual = np.mean(squared_errors)

# RMSE
rmse_manual = np.sqrt(mean_squared_error_manual)

# Print final RMSE
print(f"\nManually Computed RMSE: {rmse_manual:.4f} meters")

# -----------------------------
# 7. Plot the Results
# -----------------------------

plt.figure(figsize=(14, 7))

plt.plot(
    average_distances_df['Window_Start_Time'],
    average_distances_df['Average_Distance (m)'],
    marker='o',
    linestyle='-',
    color='blue',
    label='Measured (UWB Data)'
)

plt.plot(
    average_distances_df['Window_Start_Time'],
    average_distances_df['Ground_Truth_Distance (m)'],
    marker='s',
    linestyle='--',
    color='red',
    label='Ground Truth'
)

for start_time in average_distances_df['Window_Start_Time']:
    plt.axvline(x=start_time, color='gray', linestyle=':', linewidth=0.5)

plt.xticks(
    ticks=average_distances_df['Window_Start_Time'],
    labels=[f"{int(t)}" for t in average_distances_df['Window_Start_Time']],
    rotation=45
)

y_max = max(average_distances_df['Ground_Truth_Distance (m)'].max(), 9)
plt.yticks(
    ticks=np.arange(0, y_max + 0.4, 0.4),
    labels=[f"{gt:.1f}" for gt in np.arange(0, y_max + 0.4, 0.4)]
)

plt.xlabel(f'Start Time of {sub_window_size}-Second Sub-Interval (s)', fontsize=12)
plt.ylabel('Average Distance (m)', fontsize=12)
plt.title(f"Average UWB Distance Every {sub_window_size} Seconds ({sub_window_size}-Second Intervals) vs Ground Truth", fontsize=14)

plt.grid(True, which='both', linestyle='--', linewidth=0.5)
plt.legend()
plt.tight_layout()

# Save the plot as PDF
output_pdf = 'uwb_analysis_plot.pdf'
plt.savefig(output_pdf, format='pdf')

print(f"\nPlot has been saved as '{output_pdf}'.")
plt.show()
