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
    error_output_file = 'absolute_error.csv'
    rmse_output_file = 'rmse.txt'
    combined_plot_file = 'combined_plot.png'  # Optional: Save the combined plot as an image

    # ----------------------------- Step 2: Set Maximum Time Range in Seconds ----------------------------- #

    # Set the maximum time range (in seconds) for the plot and RMSE calculation
    max_time_seconds = 120  # Adjust this value to set the time range for your dataset

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

    # ----------------------- Step 5: Aggregate Data by Timestamp ----------------------- #

    # Function to aggregate data by timestamp by taking the mean of angle values
    def aggregate_data(df, angle_column, df_name):
        aggregated_df = df.groupby('timestamp_ns').agg({angle_column: 'mean'}).reset_index()
        print(f"✅ Aggregated {df_name}: {len(aggregated_df)} unique timestamps.")
        return aggregated_df

    encoder_agg = aggregate_data(encoder_df, 'angle_deg', "encoder data")
    screw_axis_agg = aggregate_data(screw_axis_df, 'angle_axis', "screw axis data")

    # ----------------------------- Step 6: Merge Datasets ----------------------------- #

    # Merge the two datasets on timestamp_ns using an inner join
    merged_df = pd.merge(encoder_agg, screw_axis_agg, on='timestamp_ns', how='inner')
    print(f"✅ Merged data: {len(merged_df)} matching timestamps found.")

    if merged_df.empty:
        print("❌ No matching timestamps found between encoder and screw axis data.")
        sys.exit(1)

    # ----------------------------- Step 7: Calculate Errors ----------------------------- #

    # Calculate absolute error between encoder and screw axis angles
    merged_df['absolute_error'] = (merged_df['angle_deg'] - merged_df['angle_axis']).abs()

    # Convert timestamp from nanoseconds to seconds for plotting
    merged_df['time_sec'] = merged_df['timestamp_ns'] / 1e9

    # Sort by time to ensure the plots are ordered
    merged_df = merged_df.sort_values('time_sec')

    # ------------------------- Step 8: Filter Data Based on Relative Time Range ------------------------- #

    # Get the starting timestamp (first timestamp in the dataset)
    start_time_ns = merged_df['timestamp_ns'].min()

    # Define the ending timestamp based on the max_time range
    end_time_ns = start_time_ns + max_time_ns

    # Filter the data to include only timestamps within the time range of (start_time_ns to end_time_ns)
    filtered_df = merged_df[(merged_df['timestamp_ns'] >= start_time_ns) & (merged_df['timestamp_ns'] <= end_time_ns)]

    # Check if filtered data exists
    if filtered_df.empty:
        print(f"❌ No data found within the time range of {max_time_seconds} seconds after the start time.")
        sys.exit(1)

    # ----------------------- Step 9: Save Absolute Error Data ----------------------- #

    # Save the absolute error data to a CSV file
    filtered_df[['timestamp_ns', 'time_sec', 'absolute_error']].to_csv(error_output_file, index=False)
    print(f"✅ Absolute error data saved to '{error_output_file}'")

    # ------------------------- Step 10: Calculate and Save RMSE ------------------------- #

    # Calculate RMSE for the filtered data
    rmse = np.sqrt(np.mean(filtered_df['absolute_error'] ** 2))
    print(f"✅ Root Mean Squared Error (RMSE): {rmse:.4f}")

    # Save the RMSE to a text file
    try:
        with open(rmse_output_file, 'w') as f:
            f.write(f"Root Mean Squared Error (RMSE): {rmse:.4f}\n")
        print(f"✅ RMSE value saved to '{rmse_output_file}'")
    except Exception as e:
        print(f"❌ Error writing RMSE to file: {e}")

    # ------------------------------- Step 11: Plotting ------------------------------- #

    # Create a figure for the combined plot
    plt.figure(figsize=(16, 8))

    # Plot Encoder and Screw Axis Angles
    plt.plot(filtered_df['time_sec'], filtered_df['angle_deg'], marker='o', linestyle='-', color='blue', label='Encoder Angle')
    plt.plot(filtered_df['time_sec'], filtered_df['angle_axis'], marker='s', linestyle='-', color='green', label='Screw Axis Angle')

    # Plot Absolute Error on the same figure
    plt.plot(filtered_df['time_sec'], filtered_df['absolute_error'], marker='^', linestyle='-', color='red', label='Absolute Error')

    # Set titles and labels
    plt.title(f'Encoder and Screw Axis Angles with Absolute Error (0-{max_time_seconds}s after start)', fontsize=16)
    plt.xlabel('Time (seconds)', fontsize=14)
    plt.ylabel('Angle (degrees)', fontsize=14)

    # Add legend
    plt.legend(fontsize=12)

    # Add grid
    plt.grid(True)

    # Optional: Save the plot as an image
    try:
        plt.savefig(combined_plot_file)
        print(f"✅ Combined plot saved as '{combined_plot_file}'")
    except Exception as e:
        print(f"❌ Error saving combined plot: {e}")

    # Display the plot
    plt.show()

if __name__ == "__main__":
    main()
