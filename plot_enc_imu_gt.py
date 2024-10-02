import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import csv

# File paths for the CSV files
encoder_file = 'dataset-encoder1.csv'
imu_mag_yaw_file = 'datset-imu1.csv'
error_output_file = 'yaw_encoder_error.csv'

# Initialize the plot
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))

# Set up the plot for encoder data
ax1.set_title('Encoder Angle Over Time')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Angle (degrees)')
ax1.grid(True)

# Set up the plot for IMU angle-axis data
ax2.set_title('IMU Angle Axis Over Time')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle Axis')
ax2.grid(True)

# Set up the plot for the absolute error between encoder and IMU data
ax3.set_title('Absolute Error (Encoder vs IMU)')
ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Absolute Error (degrees)')
ax3.grid(True)

# Record the start time of the program (nanoseconds)
start_time_ns = time.time_ns()

# Open a CSV file to save the absolute error data
with open(error_output_file, 'w', newline='') as csvfile:
    csvwriter = csv.writer(csvfile)
    # Write the header
    csvwriter.writerow(['timestamp_s', 'absolute_error'])

    # Function to convert timestamps to seconds from start time
    def convert_to_seconds(df, timestamp_col, start_time_ns):
        # Only consider timestamps after the starting time
        df = df[df[timestamp_col] >= start_time_ns]
        df[timestamp_col] = (df[timestamp_col] - start_time_ns) / 1e9  # Convert nanoseconds to seconds since start time
        return df

    # Update function for real-time plotting
    def update_plot(frame):
        # Read the CSV files again to get the updated data
        try:
            encoder_data_df = pd.read_csv(encoder_file)
            imu_mag_yaw_df = pd.read_csv(imu_mag_yaw_file)

            # Convert timestamps to seconds starting from the start time of the program
            encoder_data_df = convert_to_seconds(encoder_data_df, 'timestamp_ns', start_time_ns)
            imu_mag_yaw_df = convert_to_seconds(imu_mag_yaw_df, 'timestamp_ns', start_time_ns)

            # Clear the previous data from plots
            ax1.cla()
            ax2.cla()
            ax3.cla()

            # Update the encoder data plot
            ax1.plot(encoder_data_df['timestamp_ns'], abs(encoder_data_df['angle_deg']), 'r-', label='Encoder Angle')
            ax1.set_title('Encoder Angle Over Time (Since Start)')
            ax1.set_xlabel('Time (s)')
            ax1.set_ylabel('Angle (degrees)')
            ax1.grid(True)

            # Update the IMU angle-axis data plot
            ax2.plot(imu_mag_yaw_df['timestamp_ns'], abs(imu_mag_yaw_df['angle_axis']), 'g-', label='IMU Angle Axis')
            ax2.set_title('IMU Angle Axis Over Time (Since Start)')
            ax2.set_xlabel('Time (s)')
            ax2.set_ylabel('Angle Axis')
            ax2.grid(True)

            # Compute the absolute error between encoder and IMU angles
            if not encoder_data_df.empty and not imu_mag_yaw_df.empty:
                # Merge the two dataframes on the timestamp to align the data points for error calculation
                merged_df = pd.merge_asof(encoder_data_df, imu_mag_yaw_df, on='timestamp_ns', direction='nearest')

                # Calculate the absolute error
                merged_df['absolute_error'] = abs(abs(merged_df['angle_deg']) - abs(merged_df['angle_axis']))

                # Update the error plot
                ax3.plot(merged_df['timestamp_ns'], merged_df['absolute_error'], 'b-', label='Absolute Error')
                ax3.set_title('Absolute Error (Encoder vs IMU)')
                ax3.set_xlabel('Time (s)')
                ax3.set_ylabel('Absolute Error (degrees)')
                ax3.grid(True)

                # Save the absolute error to CSV
                for _, row in merged_df.iterrows():
                    csvwriter.writerow([row['timestamp_ns'], row['absolute_error']])
                csvfile.flush()  # Ensure data is written to the file immediately

        except Exception as e:
            print(f"Error reading files or updating plot: {e}")

    # Create the real-time plot animation
    ani = FuncAnimation(fig, update_plot, interval=20)  # Update every 1 second

    # Show the plot
    plt.tight_layout()
    plt.show()
