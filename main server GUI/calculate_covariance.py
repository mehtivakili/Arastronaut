import numpy as np
import pandas as pd
import os

def read_data_and_calculate_covariance(file_path):
    # Read the data from the file
    data = np.loadtxt(file_path, delimiter=' ')  # Use space as delimiter if file is space-separated
    
    # Extract the columns that represent the measurements (assuming columns 2, 3, 4 are the measurements)
    measurements = data[:, 1:]  # Skipping the first column (timestamp)
    
    # Calculate the covariance matrix
    cov_matrix = np.cov(measurements, rowvar=False)
    
    # Convert to DataFrame for easier visualization
    cov_df = pd.DataFrame(cov_matrix, columns=["X", "Y", "Z"], index=["X", "Y", "Z"])
    
    return cov_df

if __name__ == "__main__":
    # Define the file_path variable
    file_path = r"E:\aras\arastronaut\arastronaut\Arastronaut\main server GUI\acc-202408301526.csv"
    # file_path = input("./acc-202408301522.csv")
    # Optional: Print the current working directory
    print("Current Working Directory:", os.getcwd())
    
    # Call the function to calculate the covariance matrix
    covariance_matrix = read_data_and_calculate_covariance(file_path)
    
    # Print the resulting covariance matrix
    print("Covariance Matrix:")
    print(covariance_matrix)
