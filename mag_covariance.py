import numpy as np
import socket
import struct
import time
import pandas as pd

# Magnetometer calibration offsets and scales
mag_offsets = np.array([537.0, 214.0, 55.0])  # Example offsets for X, Y, Z axes
mag_scales = np.array([2510.0, 2172.0, 2356.0])  # Example scales for X, Y, Z axes

# Function to calibrate magnetometer data
def calibrate_data(data, offsets, scales):
    # Subtract the offsets and divide by the scales to normalize the data
    return (data - offsets) / scales

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# Data storage
mag_data_list = []

# Duration for data collection (in seconds)
duration = 200
end_time = time.time() + duration

print("Starting data collection for 200 seconds...")

# Collect magnetometer data for the specified duration
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

            # Calibrate magnetometer data
            mag_calibrated = calibrate_data(mag, mag_offsets, mag_scales)

            # Store calibrated magnetometer data
            mag_data_list.append(mag_calibrated)

# Convert list to a DataFrame
mag_data_df = pd.DataFrame(mag_data_list, columns=['Mag_X', 'Mag_Y', 'Mag_Z'])

# Save the calibrated data to a CSV file
mag_data_df.to_csv('200smag_data.csv', index=False)
print("Data collection complete. Data saved to '200smag_data.csv'.")

# Calculate covariance matrix of the calibrated magnetometer data
cov_matrix = mag_data_df.cov()
print("Covariance matrix of the calibrated magnetometer data:")
print(cov_matrix)

# Optionally, you can save the covariance matrix to a file
cov_matrix.to_csv('mag_cov_matrix.csv')
print("Covariance matrix saved to 'mag_cov_matrix.csv'.")
