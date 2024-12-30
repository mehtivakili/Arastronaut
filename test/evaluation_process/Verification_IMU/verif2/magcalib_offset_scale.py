import pandas as pd
import numpy as np

# Function to calculate offset and scale factor for each axis
def calculate_offset_scale_factor(data):
    offsets = data.mean(axis=0)  # Calculate the mean for each column as offset
    ranges = data.max(axis=0) - data.min(axis=0)  # Calculate the range for each column
    scale_factors = 2 / ranges  # Scale factor (assuming ideal range is [-1, 1])
    
    return offsets, scale_factors

# Load the CSV file
def load_magnetometer_data(file_path):
    df = pd.read_csv(file_path)
    return df

# Main function to read the file and calculate offsets and scale factors
def main(file_path):
    df = load_magnetometer_data(file_path)
    
    # Calculate offset and scale factor
    offsets, scale_factors = calculate_offset_scale_factor(df)
    
    # Display the results
    print("Offsets (bias) for each axis:")
    print(f"Mag_X: {offsets['Mag_X']}, Mag_Y: {offsets['Mag_Y']}, Mag_Z: {offsets['Mag_Z']}")
    
    print("\nScale factors for each axis:")
    print(f"Mag_X: {1/scale_factors['Mag_X']}, Mag_Y: {1/scale_factors['Mag_Y']}, Mag_Z: {1/scale_factors['Mag_Z']}")

if __name__ == "__main__":
    # Provide the CSV file path
    file_path = "./202smag_data.csv"
    main(file_path)
