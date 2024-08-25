import pandas as pd

# Load the CSV file
input_file = 'imu_data_with_timestamps.csv'  # Replace with your actual CSV file path
df = pd.read_csv(input_file)

# Function to find rows where data in a specific column is smaller than the previous row
def find_problematic_rows(df, column_name):
    problematic_rows = []

    for i in range(1, len(df)):
        if df[column_name].iloc[i] < df[column_name].iloc[i - 1]:
            problematic_rows.append((i-1, i))

    return problematic_rows

# Specify the column you want to check
column_to_check = 'Timestamp (ns)'  # Replace with the actual column you want to check

# Find the rows where the value is smaller than the previous row
problematic_row_indices = find_problematic_rows(df, column_to_check)

# Prepare the output in the desired format
print("Problematic rows:")
for prev_idx, curr_idx in problematic_row_indices:
    print(df.iloc[[prev_idx, curr_idx]].to_string(index=False))
    print()

