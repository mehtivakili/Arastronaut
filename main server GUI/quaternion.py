import numpy as np
from scipy.spatial.transform import Rotation as R
import socket
import struct
import time

# Madgwick Filter Parameters
beta = 0.1  # algorithm gain
q = np.array([1, 0, 0, 0])  # Initial quaternion

# Calibration matrices and offsets (will be loaded later)
acc_misalignment = None
acc_scale = None
acc_bias = None
gyro_misalignment = None
gyro_scale = None
gyro_bias = None

# Timing variables for printing at 100ms intervals
last_print_time = time.time()

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

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def madgwick_update(q, gyro, acc, dt):
    global beta
    acc = normalize(acc)

    q1, q2, q3, q4 = q
    gx, gy, gz = gyro

    f1 = 2*(q2*q4 - q1*q3) - acc[0]
    f2 = 2*(q1*q2 + q3*q4) - acc[1]
    f3 = 2*(0.5 - q2*q2 - q3*q3) - acc[2]
    
    J_11or24 = 2*q3
    J_12or23 = 2*q4
    J_13or22 = 2*q1
    J_14or21 = 2*q2
    J_32 = 2*J_14or21
    J_33 = 2*J_11or24
    
    step = np.array([
        J_14or21 * f2 - J_11or24 * f1,
        J_12or23 * f1 + J_13or22 * f2 - J_32 * f3,
        J_12or23 * f2 - J_33 * f3 - J_13or22 * f1,
        J_14or21 * f1 + J_11or24 * f2
    ])
    
    step = normalize(step)
    
    q_dot = 0.5 * np.array([
        -q2*gx - q3*gy - q4*gz,
        q1*gx + q3*gz - q4*gy,
        q1*gy - q2*gz + q4*gx,
        q1*gz + q2*gy - q3*gx
    ]) - beta * step

    q = q + q_dot * dt
    return normalize(q)

def receive_udp_data():
    global last_print_time
    UDP_IP = "0.0.0.0"
    UDP_PORT = 12346

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP packets on port {UDP_PORT}...")

    prev_time = None

    while True:
        data, addr = sock.recvfrom(4096)
        # print(f"Received packet from {addr}")

        check = "abc/"
        check_encoded = check.encode()
        parts = data.split(check_encoded)

        for part in parts:
            if len(part) == 32:
                values = struct.unpack('<q6f', part)
                Tio, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = values

                accel = np.array([accelX, accelY, accelZ])
                gyro = np.array([gyroX, gyroY, gyroZ])

                if (acc_misalignment is not None and acc_scale is not None and acc_bias is not None) and \
                   (gyro_misalignment is not None and gyro_scale is not None and gyro_bias is not None):
                    accel, gyro = apply_calibration(accel, gyro)

                timestamp = Tio / 1e9  # Convert Tio to seconds
                if prev_time is None:
                    prev_time = timestamp

                dt = timestamp - prev_time
                prev_time = timestamp

                global q
                q = madgwick_update(q, gyro, accel, dt)

                current_time = time.time()
                if current_time - last_print_time >= 0.1:  # 100ms interval
                    print(f"Calibrated Accel: {accel}, Calibrated Gyro: {gyro}")
                    print(f"Quaternion: {q}")
                    last_print_time = current_time
            else:
                if part:
                    print(f"Received packet of incorrect size: {len(part)} bytes")

if __name__ == "__main__":
    acc_calibration_file = "./calib_data/test_imu_acc.calib"
    gyro_calibration_file = "./calib_data/test_imu_gyro.calib"

    acc_misalignment, acc_scale, acc_bias = load_calibration(acc_calibration_file)
    gyro_misalignment, gyro_scale, gyro_bias = load_calibration(gyro_calibration_file)
    
    receive_udp_data()
