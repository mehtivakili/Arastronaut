import numpy as np
import socket
import struct
import time
import pandas as pd

# Magnetometer calibration offsets and scales
mag_offsets = np.array([537.0, 214.0, 55.0])  # Example offsets for X, Y, Z axes
mag_scales = np.array([2510.0, 2172.0, 2356.0])  # Example scales for X, Y, Z axes

# Define acceptable range for magnetometer data
lower_bound = -8000  # Adjust as per expected sensor range
upper_bound = 8000   # Adjust as per expected sensor range

# Function to calibrate magnetometer data
def calibrate_data(data, offsets, scales):
    # Subtract the offsets and divide by the scales to normalize the data
    return (data - offsets) / scales

# Function to filter out corrupted data
def filter_data(data, lower_bound, upper_bound):
    if np.all((data >= lower_bound) & (data <= upper_bound)):
        return True
    return False

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# File for writing data in real-time
output_file = '203smag_data.csv'
header_written = False

# Duration for data collection (in seconds)
duration = 200
end_time = time.time() + duration

print("Starting data collection for 200 seconds...")

# Open the file in write mode and start collecting data
with open(output_file, 'w') as f:
    print("hoy")
    # Start data collection loop
    while time.time() < end_time:
        data, addr = sock.recvfrom(4096)
        check = "img/"
        check_encoded = check.encode()
        parts = data.split(check_encoded)
        for part in parts:
            if len(part) == 44:  # 64-bit timestamp + 9 floats (accel, gyro, mag)
                values = struct.unpack('<q9f', part)
                magX, magY, magZ = values[7:]  # Extract magnetometer data
                
                mag = np.array([magX, magY, magZ])
                print(mag)
                # Filter the data (only process valid data)
                if filter_data(mag, lower_bound, upper_bound):
                    # Calibrate magnetometer data
                    # mag_calibrated = calibrate_data(mag, mag_offsets, mag_scales)

                    # If header is not written, write it once
                    if not header_written:
                        f.write("Mag_X,Mag_Y,Mag_Z\n")
                        header_written = True

                    # Write the calibrated data to the CSV file in real-time
                    f.write(f"{mag[0]},{mag[1]},{mag[2]}\n")

                    print(mag, end_time - time.time())  # For real-time monitoring

print(f"Data collection complete. Data saved to '{output_file}'.")

# Load the data back to calculate covariance matrix
mag_data_df = pd.read_csv(output_file)

# Calculate covariance matrix of the calibrated magnetometer data
cov_matrix = mag_data_df.cov()
print("Covariance matrix of the calibrated magnetometer data:")
print(cov_matrix)

# Optionally, save the covariance matrix to a file
cov_matrix.to_csv('mag_cov_matrix.csv')
print("Covariance matrix saved to 'mag_cov_matrix.csv'.")
