import pandas as pd

# Load the CSV file
input_file = 'imu_data_with_timestamps_nano.csv'
df = pd.read_csv(input_file)

# Convert Timestamp to start from zero and convert to seconds
initial_timestamp_ns = df['Timestamp (ns)'].iloc[0]
df['Time (s)'] = (df['Timestamp (ns)'] - initial_timestamp_ns) / 1e9

# Drop the original nanosecond timestamp column
df.drop(columns=['Timestamp (ns)'], inplace=True)

# Calculate the rate of data points per second
df['Second'] = df['Time (s)'].astype(int)
data_rate = df.groupby('Second').size().reset_index(name='Data Rate (points per second)')

# Output the modified data
output_file = 'modified_imu_data_with_rate_nano.csv'
df.to_csv(output_file, index=False)

# Output the data rate per second
data_rate_file = 'data_rate_per_second.csv'
data_rate.to_csv(data_rate_file, index=False)

print(f"Modified IMU data saved to {output_file}")
print(f"Data rate per second saved to {data_rate_file}")
