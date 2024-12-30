import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def main():
    # ----------------------------- Step 1: Define File Paths ----------------------------- #

    # Input file paths (modify these if your files have different names or locations)
    encoder_file = 'dataset-encoder3.csv'
    screw_axis_file = 'dataset-screw3.csv'

    # Output file paths
    rmse_output_file = 'rmse_output.txt'  # File to log RMSE value
    plot_output_file = 'imu_screw_angle_verification.pdf'  # PDF output

    # ----------------------------- Step 2: Set Maximum Time Range in Seconds ----------------------------- #

    # Set the maximum time range (in seconds) for the plot and error calculation
    max_time_seconds = 300  # Adjust this value to set the time range for your dataset

    # Convert max_time_seconds to nanoseconds
    max_time_ns = max_time_seconds * 1e9  # Convert seconds to nanoseconds

    # ----------------------------- Step 3: Read the CSV Data ----------------------------- #

    # Function to read CSV with error handling
    def read_csv_file(file_path, file_description):
        if not os.path.isfile(file_path):
            print(f"❌ Error: {file_description} file '{file_path}' does not exist.")
            sys.exit(1)
        try:
            df = pd.read_csv(file_path)
            print(f"✅ Successfully read '{file_path}'")
            return df
        except Exception as e:
            print(f"❌ Error reading '{file_path}': {e}")
            sys.exit(1)

    # Read the encoder and screw axis data
    encoder_df = read_csv_file(encoder_file, "Encoder data")
    screw_axis_df = read_csv_file(screw_axis_file, "Screw axis data")

    # ---------------------- Step 4: Data Type Conversion and Cleaning ---------------------- #

    # Convert timestamp_ns to integer (if not already)
    try:
        encoder_df['timestamp_ns'] = encoder_df['timestamp_ns'].astype(int)
        screw_axis_df['timestamp_ns'] = screw_axis_df['timestamp_ns'].astype(int)
    except KeyError as e:
        print(f"❌ Error: Missing column in one of the CSV files: {e}")
        sys.exit(1)
    except ValueError as e:
        print(f"❌ Error: Non-integer values found in 'timestamp_ns' column: {e}")
        sys.exit(1)

    # Ensure all angle values are positive by taking their absolute values and scale by two
    if 'angle_deg' in encoder_df.columns:
        encoder_df['angle_deg'] = encoder_df['angle_deg'].abs() * 2
    else:
        print("❌ Error: 'angle_deg' column not found in encoder data.")
        sys.exit(1)

    if 'angle_axis' in screw_axis_df.columns:
        screw_axis_df['angle_axis'] = screw_axis_df['angle_axis'].abs() * 2
    else:
        print("❌ Error: 'angle_axis' column not found in screw axis data.")
        sys.exit(1)

    # Check for missing values and handle them
    def handle_missing_values(df, df_name):
        if df.isnull().values.any():
            print(f"⚠️ Warning: Missing values detected in {df_name}. Dropping missing entries.")
            df = df.dropna()
        else:
            print(f"✅ No missing values in {df_name}.")
        return df

    encoder_df = handle_missing_values(encoder_df, "encoder data")
    screw_axis_df = handle_missing_values(screw_axis_df, "screw axis data")

    # ----------------------------- Step 5: Synchronize Time Across Both Datasets ----------------------------- #

    # Find the minimum timestamp across both datasets
    global_min_time_ns = min(encoder_df['timestamp_ns'].min(), screw_axis_df['timestamp_ns'].min())

    # Convert timestamp from nanoseconds to seconds for both datasets based on the common minimum timestamp
    encoder_df['time_sec'] = (encoder_df['timestamp_ns'] - global_min_time_ns) / 1e9
    screw_axis_df['time_sec'] = (screw_axis_df['timestamp_ns'] - global_min_time_ns) / 1e9

    # Filter data to the desired time range for screw_axis_df only (we will plot the full encoder data)
    start_time_ns = encoder_df['timestamp_ns'].min()
    end_time_ns = start_time_ns + max_time_ns
    filtered_screw_axis_df = screw_axis_df[
        (screw_axis_df['timestamp_ns'] >= start_time_ns) & (screw_axis_df['timestamp_ns'] <= end_time_ns)
    ]

    # ----------------------------- Step 6: Calculate Error ----------------------------- #

    # Create a list to store the error data
    error_list = []

    for index, screw_row in filtered_screw_axis_df.iterrows():
        # Find the nearest timestamp in the encoder data
        nearest_encoder_idx = (encoder_df['timestamp_ns'] - screw_row['timestamp_ns']).abs().idxmin()
        nearest_encoder_row = encoder_df.loc[nearest_encoder_idx]

        # Calculate the absolute error between screw axis and nearest encoder angles
        error = abs(screw_row['angle_axis'] - nearest_encoder_row['angle_deg'])

        # Handle potential angle wrapping (if error > 180 after scaling, adjust it)
        if error > 180:
            error = 360 - error

        # Store the error data along with the timestamp
        error_list.append({
            'time_sec': screw_row['time_sec'],
            'error': error
        })

    # Convert the list to a dataframe
    error_df = pd.DataFrame(error_list)

    # ----------------------------- Step 7: Calculate RMSE ----------------------------- #
    
    # Calculate the RMSE from the error values
    rmse = np.sqrt(np.mean(error_df['error'] ** 2))
    print(f"✅ Root Mean Squared Error (RMSE): {rmse:.4f}")

    # Write the RMSE to a file
    try:
        with open(rmse_output_file, 'w') as f:
            f.write(f"Root Mean Squared Error (RMSE): {rmse:.4f}\n")
        print(f"✅ RMSE value saved to '{rmse_output_file}'")
    except Exception as e:
        print(f"❌ Error writing RMSE to file: {e}")

    # ----------------------------- Step 8: Plot Full Encoder Data, Screw Axis Data, and Error ----------------------------- #

    # Create a figure
    plt.figure(figsize=(16, 8))

    # Plot Full Encoder Angle
    plt.plot(
        encoder_df['time_sec'],
        encoder_df['angle_deg'],
        linestyle='-',
        color='black',
        linewidth=2,
        label='Encoder Angle'
    )

    # Plot Screw Axis Angle (filtered for the time range)
    plt.plot(
        filtered_screw_axis_df['time_sec'],
        filtered_screw_axis_df['angle_axis'],
        linestyle='-',
        color='red',
        linewidth=2,
        label='IMU Angle'
    )

    # Plot Error (optional)
    # plt.plot(
    #     error_df['time_sec'],
    #     error_df['error'],
    #     linestyle='-',
    #     color='blue',
    #     linewidth=2,
    #     label='Absolute Error'
    # )

    # Set titles and labels with increased font sizes
    plt.title('IMU Screw Angle Verification', fontsize=40)
    plt.xlabel('Time (seconds)', fontsize=28)
    plt.ylabel('Angle (degrees)', fontsize=28)

    # Set tick parameters with increased font sizes
    plt.xticks(fontsize=24)
    plt.yticks(fontsize=24)

    # Set y-axis limits from 0 to 360
    plt.ylim(0, 360)

    # Add legend with increased font size at the top-left corner
    plt.legend(loc='upper left', fontsize=24)

    # Add grid
    plt.grid(True)

    # Save the plot as a PDF file
    plt.savefig(plot_output_file, format='pdf')
    print(f"✅ Plot saved as PDF at '{plot_output_file}'")

    # Display the plot
    plt.show()

if __name__ == "__main__":
    main()
