import pandas as pd
import matplotlib.pyplot as plt
import sys
import os

def main():
    # ----------------------------- Step 1: Define File Paths ----------------------------- #

    # Input file paths (modify these if your files have different names or locations)
    encoder_file = 'dataset-encoder3.csv'
    screw_axis_file = 'dataset-screw3.csv'

    # ----------------------------- Step 2: Set Maximum Time Range in Seconds ----------------------------- #

    # Set the maximum time range (in seconds) for the plot
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

    # Ensure all angle values are positive by taking their absolute values
    if 'angle_deg' in encoder_df.columns:
        encoder_df['angle_deg'] = encoder_df['angle_deg'].abs()
    else:
        print("❌ Error: 'angle_deg' column not found in encoder data.")
        sys.exit(1)

    if 'angle_axis' in screw_axis_df.columns:
        screw_axis_df['angle_axis'] = screw_axis_df['angle_axis'].abs()
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

    # ----------------------------- Step 5: Prepare Data for Plotting ----------------------------- #

    # Convert timestamp from nanoseconds to seconds for plotting
    encoder_df['time_sec'] = encoder_df['timestamp_ns'] / 1e9
    screw_axis_df['time_sec'] = screw_axis_df['timestamp_ns'] / 1e9

    # Filter data to the desired time range
    start_time_ns = encoder_df['timestamp_ns'].min()
    end_time_ns = start_time_ns + max_time_ns
    filtered_encoder_df = encoder_df[(encoder_df['timestamp_ns'] >= start_time_ns) & (encoder_df['timestamp_ns'] <= end_time_ns)]
    filtered_screw_axis_df = screw_axis_df[(screw_axis_df['timestamp_ns'] >= start_time_ns) & (screw_axis_df['timestamp_ns'] <= end_time_ns)]

    # ----------------------------- Step 6: Plot Both Data in Two Subplots in a Single Window ----------------------------- #

    # Create a figure with 2 subplots (vertically aligned)
    fig, axs = plt.subplots(2, 1, figsize=(16, 12))

    # Plot Encoder Angle on the first subplot
    axs[0].plot(filtered_encoder_df['time_sec'], filtered_encoder_df['angle_deg'], marker='o', linestyle='-', color='blue', label='Encoder Angle')
    axs[0].set_title(f'Encoder Angle Over Time (0-{max_time_seconds}s)', fontsize=16)
    axs[0].set_xlabel('Time (seconds)', fontsize=14)
    axs[0].set_ylabel('Angle (degrees)', fontsize=14)
    axs[0].grid(True)
    axs[0].legend(fontsize=12)

    # Plot Screw Axis Angle on the second subplot
    axs[1].plot(filtered_screw_axis_df['time_sec'], filtered_screw_axis_df['angle_axis'], marker='s', linestyle='-', color='green', label='Screw Axis Angle')
    # axs[1].set_title(f'Screw Axis Angle Over Time (0-{max_time_seconds}s)', fontsize=16)
    axs[1].set_xlabel('Time (seconds)', fontsize=14)
    axs[1].set_ylabel('Angle (degrees)', fontsize=14)
    axs[1].grid(True)
    axs[1].legend(fontsize=12)

    # Adjust layout for better spacing
    plt.tight_layout()

    # Display the plots
    plt.show()

if __name__ == "__main__":
    main()
