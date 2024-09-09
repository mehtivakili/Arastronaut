import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def calculate_axis_error(encoder_csv_path, rpq_csv_path, output_axis, output_csv_path):
    # Load encoder data, skip the header row if needed
    encoder_data = pd.read_csv(encoder_csv_path, header=None, names=['Timestamp', 'Encoder'])
    encoder_data['Timestamp'] = pd.to_numeric(encoder_data['Timestamp'], errors='coerce')
    encoder_data['Encoder'] = pd.to_numeric(encoder_data['Encoder'], errors='coerce')
    encoder_data = encoder_data.dropna()  # Drop rows with NaN

    # Load roll/pitch/yaw data, skip the header row if needed
    rpq_data = pd.read_csv(rpq_csv_path, header=None, names=['Timestamp', 'Roll', 'Pitch', 'Yaw'])
    rpq_data['Timestamp'] = pd.to_numeric(rpq_data['Timestamp'], errors='coerce')
    rpq_data['Roll'] = pd.to_numeric(rpq_data['Roll'], errors='coerce')
    rpq_data['Pitch'] = pd.to_numeric(rpq_data['Pitch'], errors='coerce')
    rpq_data['Yaw'] = pd.to_numeric(rpq_data['Yaw'], errors='coerce')
    rpq_data = rpq_data.dropna()  # Drop rows with NaN

    # Select the axis (roll, pitch, or yaw) based on user input
    if output_axis.lower() == 'roll':
        axis_data = rpq_data['Roll']
    elif output_axis.lower() == 'pitch':
        axis_data = rpq_data['Pitch']
    elif output_axis.lower() == 'yaw':
        axis_data = rpq_data['Yaw']
    else:
        raise ValueError("Invalid axis. Choose 'roll', 'pitch', or 'yaw'.")

    # Create a new dataframe for storing the error
    error_df = pd.DataFrame(columns=['Timestamp', f'{output_axis.capitalize()}_Error'])

    # Convert timestamps to seconds relative to the first timestamp
    encoder_data['Timestamp'] -= encoder_data['Timestamp'].min()
    rpq_data['Timestamp'] -= rpq_data['Timestamp'].min()

    # Iterate over encoder data and find the closest matching time in rpq_data
    for i, encoder_row in encoder_data.iterrows():
        timestamp = encoder_row['Timestamp']
        encoder_value = encoder_row['Encoder']
        
        # Find the closest timestamp in rpq_data
        closest_row = rpq_data.iloc[(rpq_data['Timestamp'] - timestamp).abs().argmin()]
        closest_timestamp = closest_row['Timestamp']
        axis_value = closest_row[output_axis.capitalize()]

        # Make sure both encoder_value and axis_value are numeric
        if pd.notna(encoder_value) and pd.notna(axis_value):
            # Calculate the error (difference between encoder and axis value)
            error = encoder_value - axis_value

            # Append the error and timestamp to the error_df
            new_row = pd.DataFrame({'Timestamp': [closest_timestamp], f'{output_axis.capitalize()}_Error': [error]})
            error_df = pd.concat([error_df, new_row], ignore_index=True)

    # Save the error data to the output CSV file
    error_df.to_csv(output_csv_path, index=False)

    # Plot the error
    plt.plot(error_df['Timestamp'], error_df[f'{output_axis.capitalize()}_Error'], label=f'{output_axis.capitalize()} Error')
    plt.xlabel('Time (seconds)')
    plt.ylabel(f'{output_axis.capitalize()} Error (degrees)')
    plt.title(f'{output_axis.capitalize()} Error over Time')
    plt.legend()
    plt.show()

# Example usage:
output_axis = 'yaw'  # Choose 'roll', 'pitch', or 'yaw'
encoder_csv_path = 'encoder_data.csv'  # Update path
rpq_csv_path = 'rpq_data.csv'  # Update path
output_csv_path = f'{output_axis}_error_output.csv'

calculate_axis_error(encoder_csv_path, rpq_csv_path, output_axis, output_csv_path)
