import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

# -----------------------------
# 1. Read Data from CSV
# -----------------------------

csv_file_path = 'time_distance_data1.csv'

try:
    df = pd.read_csv(csv_file_path)
    df = df[df['Time (s)'] <= 382]
except FileNotFoundError:
    print(f"Error: The file '{csv_file_path}' was not found.")
    sys.exit(1)
except pd.errors.EmptyDataError:
    print(f"Error: The file '{csv_file_path}' is empty.")
    sys.exit(1)
except pd.errors.ParserError:
    print(f"Error: The file '{csv_file_path}' does not appear to be in CSV format.")
    sys.exit(1)

# Verify that required columns exist
required_columns = ['Time (s)', 'Distance (m)']
if not all(col in df.columns for col in required_columns):
    print(f"Error: The CSV file must contain the following columns: {required_columns}")
    sys.exit(1)

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
# 3. Assign 20-Second Window Numbers
# -----------------------------

total_window_size = 20  # seconds

# Calculate window number for each data point
df['Window_Number'] = (df['Time (s)'] // total_window_size).astype(int) + 1  # Windows start at 1

# -----------------------------
# 4. Detect and Remove Outliers Within Each Window
# -----------------------------

def remove_outliers(group, z_thresh=2):
    """
    Remove outliers from a group based on z-score threshold.
    """
    mean = group['Distance (m)'].mean()
    std = group['Distance (m)'].std()
    if std == 0:
        # If standard deviation is zero, no outliers can be detected
        return group
    # Calculate z-scores
    group['z_score'] = (group['Distance (m)'] - mean) / std
    # Keep only data points within the threshold
    cleaned_group = group[group['z_score'].abs() <= z_thresh].copy()
    return cleaned_group

# Apply outlier removal for each window
df_cleaned = df.groupby('Window_Number').apply(remove_outliers).reset_index(drop=True)

# Drop the temporary 'z_score' column
df_cleaned = df_cleaned.drop(columns=['z_score'])

# Identify and report outliers
outliers = df[~df.index.isin(df_cleaned.index)]
print(f"\nNumber of outliers detected and removed: {len(outliers)}")
if not outliers.empty:
    print("Outliers:")
    print(outliers)

# -----------------------------
# 5. Calculate Mean Distance in Each 20-Second Window
# -----------------------------

# Compute mean for each 20-second window
window_means = df_cleaned.groupby('Window_Number')['Distance (m)'].mean().reset_index()
window_means['Window_Start_Time'] = (window_means['Window_Number'] - 1) * total_window_size
window_means['Window_End_Time'] = window_means['Window_Start_Time'] + total_window_size

# -----------------------------
# 6. Plot the Cleaned Data, Outliers, and Window Means as Steps
# -----------------------------

# -----------------------------
# 6.a. Apply Time Offset
# -----------------------------

# Apply a 40-second offset to the 'Time (s)' column
offset_seconds = 40
df_cleaned['Time (s)'] = df_cleaned['Time (s)'] - offset_seconds
outliers['Time (s)'] = outliers['Time (s)'] - offset_seconds
window_means['Window_Start_Time'] = window_means['Window_Start_Time'] - offset_seconds
window_means['Window_End_Time'] = window_means['Window_End_Time'] - offset_seconds

# Remove any data points with negative time after offset
df_cleaned = df_cleaned[df_cleaned['Time (s)'] >= 0].reset_index(drop=True)
outliers = outliers[outliers['Time (s)'] >= 0].reset_index(drop=True)
window_means = window_means[window_means['Window_End_Time'] > 0].reset_index(drop=True)

# -----------------------------
# 6.b. Plotting with Enhanced Font Sizes
# -----------------------------

plt.figure(figsize=(16, 8))  # Increased figure size for better readability

# Plot outliers in pale red
plt.plot(
    outliers['Time (s)'],
    outliers['Distance (m)'],
    linestyle='-',
    color='lightcoral',
    label='Outliers (Removed)',
    alpha=0.6
)

# Plot cleaned data in blue
plt.plot(
    df_cleaned['Time (s)'],
    df_cleaned['Distance (m)'],
    linestyle='-',
    color='blue',
    label='Measured (Without Outliers)',
    alpha=0.6
)

# Plot mean as steps (constant lines for each window)
for _, row in window_means.iterrows():
    plt.hlines(
        y=row['Distance (m)'],
        xmin=row['Window_Start_Time'],
        xmax=row['Window_End_Time'],
        colors='orange',
        label='Mean in Each 20-Second Window' if row['Window_Start_Time'] == window_means['Window_Start_Time'].min() else "",
        linestyles='solid',
    )

# Set titles and labels with increased font sizes
plt.title('Distance Over Time (With Outliers Highlighted and Constant Mean in 20-Second Windows)', fontsize=28)
plt.xlabel('Time (s)', fontsize=24)
plt.ylabel('Distance (m)', fontsize=24)

# Set tick parameters with increased font sizes
plt.xticks(fontsize=20)
plt.yticks(fontsize=20)

# Add grid
plt.grid(True, which='both', linestyle='--', linewidth=0.5)

# Add legend with increased font size
plt.legend(fontsize=20, loc='upper left')

plt.tight_layout()

# Save the cleaned data plot as PDF
output_pdf_cleaned = 'distance_plot_cleaned_with_outliers_and_constant_mean.pdf'
plt.savefig(output_pdf_cleaned, format='pdf')
print(f"\nCleaned data plot (with outliers highlighted and constant window means) has been saved as '{output_pdf_cleaned}'.")
plt.show()

# -----------------------------
# 7. Continue with the Rest of the Analysis on the Cleaned Data
# -----------------------------

# Update the dataframe to the cleaned data
df = df_cleaned.copy()

# -----------------------------
# 8. Define 20-Second Windows and 15-Second Middle Sub-Intervals
# -----------------------------

sub_window_size = 15  # seconds

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
    print(f"Window {idx + 1}: {start:.1f} - {start + total_window_size:.1f} s | Sub-Window: {sub_start:.1f} - {sub_end:.1f} s")

# -----------------------------
# 9. Calculate Average Distance for Each 15-Second Sub-Interval
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
average_distances_df = average_distances_df.drop(average_distances_df.index[-1])

print("\nAverage Distances for Each 15-Second Middle Sub-Interval:")
print(average_distances_df)

# -----------------------------
# 10. Generate Ground Truth Data
# -----------------------------

ground_truth_distances = []

for idx, row in average_distances_df.iterrows():
    window_end_time = row['Window_End_Time']
    if window_end_time <= 405:
        gt_distance = 0.4 * (row['Window_Number'] - 1) + 1.2  # Adjusted based on offset
    else:
        gt_distance = 8.6
    ground_truth_distances.append(gt_distance)

average_distances_df['Ground_Truth_Distance (m)'] = ground_truth_distances

print("\nGround Truth Distances:")
print(average_distances_df[['Window_Number', 'Window_Start_Time', 'Ground_Truth_Distance (m)']])

# -----------------------------
# 11. Recalculate RMSE between Measured and Ground Truth Distances (Using Cleaned Data)
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
    print(f"Step {index+1}: Measured = {measured:.2f}, Ground Truth = {ground_truth:.2f}, "
          f"Difference = {difference:.4f}, Squared Error = {squared_error:.4f}")

# Mean of squared errors
mean_squared_error_manual = np.mean(squared_errors)

# RMSE
rmse_manual = np.sqrt(mean_squared_error_manual)

# Print final RMSE
print(f"\nManually Computed RMSE (with cleaned data): {rmse_manual:.4f} meters")

# -----------------------------
# 12. Plot the Final Results (Keep markers and lines for UWB and Ground Truth)
# -----------------------------

plt.figure(figsize=(16, 8))  # Increased figure size for better readability

# Plot the UWB Data with 'o' markers and lines
plt.plot(
    average_distances_df['Window_Start_Time'],
    average_distances_df['Average_Distance (m)'],
    marker='o',
    linestyle='-',
    color='blue',
    label='Measured (UWB Data)'
)

# Plot the Ground Truth with 'o' markers and dashed lines
plt.plot(
    average_distances_df['Window_Start_Time'],
    average_distances_df['Ground_Truth_Distance (m)'],
    marker='o',
    linestyle='--',
    color='red',
    label='Ground Truth'
)

# Add vertical lines for each window
for start_time in average_distances_df['Window_Start_Time']:
    plt.axvline(x=start_time, color='gray', linestyle=':', linewidth=0.5)

# -----------------------------
# 12.a. Fit a Linear Regression Model to the UWB Data
# -----------------------------

# Remove NaN values for fitting
fit_df = average_distances_df.dropna(subset=['Window_Start_Time', 'Average_Distance (m)'])

# Perform linear regression using numpy
slope, intercept = np.polyfit(fit_df['Window_Start_Time'], fit_df['Average_Distance (m)'], 1)
linear_fit = slope * fit_df['Window_Start_Time'] + intercept

# Plot the linear fit
plt.plot(
    fit_df['Window_Start_Time'],
    linear_fit,
    linestyle='-',
    color='purple',
    label=f'UWB Measurement Model\n$y = {slope:.4f}x + {intercept:.4f}$'
)

# Add labels and title with increased font sizes
plt.xlabel(f'Start Time of {sub_window_size}-Second Sub-Interval (s)', fontsize=24)
plt.ylabel('Average Distance (m)', fontsize=24)
plt.title(f"Average UWB Distance Every {sub_window_size} Seconds ({sub_window_size}-Second Intervals) vs Ground Truth", fontsize=28)

# Customize tick labels with increased font sizes
plt.xticks(
    ticks=average_distances_df['Window_Start_Time'],
    labels=[f"{int(t)}" for t in average_distances_df['Window_Start_Time']],
    rotation=45,
    fontsize=20  # Increased font size for x-axis tick labels
)
y_max = max(average_distances_df['Ground_Truth_Distance (m)'].max(), 9)
plt.yticks(
    ticks=np.arange(0, y_max + 0.4, 0.4),
    labels=[f"{gt:.1f}" for gt in np.arange(0, y_max + 0.4, 0.4)],
    fontsize=20  # Increased font size for y-axis tick labels
)

# Add grid
plt.grid(True, which='both', linestyle='--', linewidth=0.5)

# Add legend with increased font size
plt.legend(fontsize=20)

plt.tight_layout()

# Save the final plot as PDF
output_pdf = 'uwb_analysis_plot_with_markers_and_linear_fit.pdf'
plt.savefig(output_pdf, format='pdf')

print(f"\nFinal analysis plot (with markers, lines, and linear fit) has been saved as '{output_pdf}'.")
plt.show()

# -----------------------------
# 13. Print the Linear Fit Equation
# -----------------------------

print(f"\nLinear Fit Equation for UWB Measurement Model:")
print(f"y = {slope:.4f}x + {intercept:.4f} meters")
