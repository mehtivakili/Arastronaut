import socket
import struct
import numpy as np
import time

# Calibration matrices and offsets (will be loaded from files)
acc_misalignment = None
acc_scale = None
acc_bias = None
gyro_misalignment = None
gyro_scale = None
gyro_bias = None

# Covariance matrices for accelerometer and gyroscope
acc_cov_matrix = np.array([
    [0.000716, 0.000058, -0.000001],
    [0.000058, 0.000830, -0.000028],
    [-0.000001, -0.000028, 0.000788]
])

gyro_cov_matrix = np.array([
    [1.810157e-06, -1.308623e-07, -8.375072e-08],
    [-1.308623e-07, 2.236266e-06, 1.143754e-07],
    [-8.375072e-08, 1.143754e-07, 8.101775e-07]
])

# Initial Kalman filter states
acc_P = np.eye(3)  # Error covariance matrix for accelerometer
gyro_P = np.eye(3)  # Error covariance matrix for gyroscope

# Measurement noise covariance (can be adjusted)
R_acc = np.eye(3) * 0.01
R_gyro = np.eye(3) * 0.01

# Initial states
acc_x = np.zeros((3, 1))
gyro_x = np.zeros((3, 1))

def load_calibration(file):
    with open(file, 'r') as f:
        lines = f.readlines()

    # Parse misalignment matrix
    misalignment = np.array([
        [float(x) for x in lines[0].split()],
        [float(x) for x in lines[1].split()],
        [float(x) for x in lines[2].split()]
    ])

    # Parse scale matrix
    scale = np.array([
        [float(x) for x in lines[4].split()],
        [float(x) for x in lines[5].split()],
        [float(x) for x in lines[6].split()]
    ])

    # Parse bias vector
    bias = np.array([
        float(lines[8].split()[0]),
        float(lines[9].split()[0]),
        float(lines[10].split()[0])
    ])

    return misalignment, scale, bias

def apply_calibration(accel, gyro):
    global acc_misalignment, acc_scale, acc_bias
    global gyro_misalignment, gyro_scale, gyro_bias

    # Apply calibration to accelerometer data
    accel_corrected = np.dot(acc_misalignment, accel - acc_bias)
    accel_calibrated = np.dot(acc_scale, accel_corrected)
    
    # Apply calibration to gyroscope data
    gyro_corrected = np.dot(gyro_misalignment, gyro - gyro_bias)
    gyro_calibrated = np.dot(gyro_scale, gyro_corrected)
    
    return accel_calibrated, gyro_calibrated

def kalman_filter(z, x, P, Q, R):
    # Prediction step
    x_prior = x  # Assuming constant model, x(k+1|k) = x(k|k)
    P_prior = P + Q

    # Update step
    y = z.reshape((3, 1)) - x_prior
    S = P_prior + R
    K = np.dot(P_prior, np.linalg.inv(S))
    x = x_prior + np.dot(K, y)
    P = np.dot((np.eye(3) - K), P_prior)

    return x, P

def receive_udp_data():
    global acc_x, acc_P, gyro_x, gyro_P
    UDP_IP = "0.0.0.0"
    UDP_PORT = 12346

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow the socket to reuse the address
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP packets on port {UDP_PORT}...")

    while True:
        data, addr = sock.recvfrom(4096)
        check = "abc/"
        check_encoded = check.encode()
        parts = data.split(check_encoded)

        for part in parts:
            if len(part) == 32:
                values = struct.unpack('<q6f', part)
                _, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = values

                accel = np.array([accelX, accelY, accelZ])
                gyro = np.array([gyroX, gyroY, gyroZ])

                if (acc_misalignment is not None and acc_scale is not None and acc_bias is not None) and \
                   (gyro_misalignment is not None and gyro_scale is not None and gyro_bias is not None):
                    accel, gyro = apply_calibration(accel, gyro)

                # Apply Kalman filter to accelerometer data
                acc_x, acc_P = kalman_filter(accel, acc_x, acc_P, acc_cov_matrix, R_acc)

                # Apply Kalman filter to gyroscope data
                gyro_x, gyro_P = kalman_filter(gyro, gyro_x, gyro_P, gyro_cov_matrix, R_gyro)

                print(f"Kalman Filtered Accel: {acc_x.T}")
                print(f"Kalman Filtered Gyro: {gyro_x.T}")
            else:
                if part:
                    print(f"Received packet of incorrect size: {len(part)} bytes")

if __name__ == "__main__":
    acc_calibration_file = "./main server GUI/calib_data/test_imu_acc.calib"
    gyro_calibration_file = "./main server GUI/calib_data/test_imu_gyro.calib"

    acc_misalignment, acc_scale, acc_bias = load_calibration(acc_calibration_file)
    gyro_misalignment, gyro_scale, gyro_bias = load_calibration(gyro_calibration_file)
    
    receive_udp_data()