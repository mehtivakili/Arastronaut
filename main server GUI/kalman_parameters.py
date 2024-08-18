import socket
import struct
import numpy as np
import csv
import logging
import time
import matplotlib.pyplot as plt

# Set logging to WARNING to suppress debug messages
logging.basicConfig(level=logging.WARNING)
# UDP settings
UDP_IP = "192.168.4.100"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# File paths for CSV files
imu_csv_file = "imu_data.csv"
uwb_csv_file = "uwb_data.csv"

# Create CSV files and write headers
with open(imu_csv_file, mode='w', newline='') as imu_file, open(uwb_csv_file, mode='w', newline='') as uwb_file:
    imu_writer = csv.writer(imu_file)
    uwb_writer = csv.writer(uwb_file)
    imu_writer.writerow(["timestamp", "accelX", "accelY", "accelZ", "gyroX", "gyroY", "gyroZ"])
    uwb_writer.writerow(["timestamp", "address", "distance"])

# Parameters
max_data_points = 500  # Number of data points to collect
imu_data = []
uwb_data = []

# Calibration parameters
calibration_enabled = True
Ta = np.array([[1, -0.00546066, 0.00101399], [0, 1, 0.00141895], [0, 0, 1]])
Ka = np.array([[0.00358347, 0, 0], [0, 0.00358133, 0], [0, 0, 0.00359205]])
Tg = np.array([[1, -0.00614889, -0.000546488], [0.0102258, 1, 0.000838491], [0.00412113, 0.0020154, 1]])
Kg = np.array([[0.000531972, 0, 0], [0, 0.000531541, 0], [0, 0, 0.000531]])
acce_bias = np.array([-8.28051, -4.6756, -0.870355])
gyro_bias = np.array([4.53855, 4.001, -1.9779])

def calibrate_imu(accel, gyro):
    accel = np.array(accel)
    gyro = np.array(gyro)
    
    acce_calibrated = Ka @ (Ta @ (accel - acce_bias))
    gyro_calibrated = Kg @ (Tg @ (gyro - gyro_bias))
    
    return acce_calibrated, gyro_calibrated

# Collect data
while len(imu_data) < max_data_points or len(uwb_data) < max_data_points:
    data, addr = sock.recvfrom(1024)
    if len(data) > 0:
        if data.startswith(b'abc/'):
            timestamp = time.time()
            accelX, accelY, accelZ = struct.unpack('fff', data[4:16])
            gyroX, gyroY, gyroZ = struct.unpack('fff', data[16:28])
            accel = (accelX, accelY, accelZ)
            gyro = (gyroX, gyroY, gyroZ)
            
            if calibration_enabled:
                accel, gyro = calibrate_imu(accel, gyro)
            
            imu_data.append([timestamp, *accel, *gyro])
            with open(imu_csv_file, mode='a', newline='') as imu_file:
                imu_writer = csv.writer(imu_file)
                imu_writer.writerow([timestamp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ])
            
            logging.debug(f"IMU Data: Accel=({accelX},{accelY},{accelZ}), Gyro=({gyroX},{gyroY},{gyroZ})")
        
        elif data.startswith(b'cba/'):
            timestamp = time.time()
            address, distance = struct.unpack('ff', data[4:12])
            uwb_data.append([timestamp, address, distance])
            with open(uwb_csv_file, mode='a', newline='') as uwb_file:
                uwb_writer = csv.writer(uwb_file)
                uwb_writer.writerow([timestamp, address, distance])
            
            logging.debug(f"UWB Data: Address={address}, Distance={distance}")

# Stop UDP collection after reaching the required data points
sock.close()

# Process the collected data to calculate noise covariance and biases

imu_data_accel = np.array([row[1:4] for row in imu_data])
imu_data_gyro = np.array([row[4:7] for row in imu_data])
uwb_distances = np.array([row[2] for row in uwb_data])

# Calculate noise covariance for the accelerometer and gyroscope
Q_accel = np.cov(imu_data_accel, rowvar=False)
Q_gyro = np.cov(imu_data_gyro, rowvar=False)
R_uwb = np.cov(uwb_distances, rowvar=False)

print(f"Accelerometer noise covariance (Q_accel):\n{Q_accel}")
print(f"Gyroscope noise covariance (Q_gyro):\n{Q_gyro}")
print(f"UWB measurement noise covariance (R_uwb):\n{R_uwb}")
print
# Estimate biases as the mean of the measured values
accel_bias_estimate = np.mean(imu_data_accel, axis=0)
gyro_bias_estimate = np.mean(imu_data_gyro, axis=0)
uwb_bias_estimate = np.mean(uwb_distances, axis=0)
 
print(f"Estimated accelerometer bias: {accel_bias_estimate}")
print(f"Estimated gyroscope bias: {gyro_bias_estimate}")
print(f"Estimated UWB distance bias: {uwb_bias_estimate}")

# Print final covariance matrices for use in the Kalman Filter
print("Final Covariance Matrices for Kalman Filter:")
print("Process Noise Covariance Q (Combined Accel and Gyro):")
Q_combined = np.block([
    [Q_accel, np.zeros((3, 3))],
    [np.zeros((3, 3)), Q_gyro]
])
print(Q_combined)
print("\nMeasurement Noise Covariance R (UWB):")
print(R_uwb)

# Plotting the data
time_stamps = [row[0] for row in imu_data]

# Plot accelerometer data
plt.figure(figsize=(15, 10))
plt.subplot(3, 1, 1)
plt.plot(time_stamps, imu_data_accel[:, 0], label="Accel X")
plt.plot(time_stamps, imu_data_accel[:, 1], label="Accel Y")
plt.plot(time_stamps, imu_data_accel[:, 2], label="Accel Z")
plt.title('Accelerometer Data')
plt.xlabel('Time')
plt.ylabel('Acceleration (m/s^2)')
plt.legend()

# Plot gyroscope data
plt.subplot(3, 1, 2)
plt.plot(time_stamps, imu_data_gyro[:, 0], label="Gyro X")
plt.plot(time_stamps, imu_data_gyro[:, 1], label="Gyro Y")
plt.plot(time_stamps, imu_data_gyro[:, 2], label="Gyro Z")
plt.title('Gyroscope Data')
plt.xlabel('Time')
plt.ylabel('Angular Velocity (rad/s)')
plt.legend()

# Plot UWB distance data
plt.subplot(3, 1, 3)
plt.plot(time_stamps[:len(uwb_distances)], uwb_distances, label="UWB Distance")
plt.title('UWB Distance Data')
plt.xlabel('Time')
plt.ylabel('Distance (m)')
plt.legend()

# Show covariance and bias estimates on the plots
textstr = f'Q_accel: {np.round(Q_accel, 3)}\nQ_gyro: {np.round(Q_gyro, 3)}\nR_uwb: {np.round(R_uwb, 3)}\n' \
          f'Accel Bias: {np.round(accel_bias_estimate, 3)}\nGyro Bias: {np.round(gyro_bias_estimate, 3)}\n' \
          f'UWB Bias: {np.round(uwb_bias_estimate, 3)}'

plt.gcf().text(0.02, 0.5, textstr, fontsize=12, verticalalignment='center')

plt.tight_layout()
plt.show()
