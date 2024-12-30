import socket
import struct
import numpy as np
import time
from scipy.spatial.transform import Rotation as R

# Calibration matrices and offsets (will be loaded from files)
acc_misalignment = None
acc_scale = None
acc_bias = None
gyro_misalignment = None
gyro_scale = None
gyro_bias = None

# Complementary Filter Parameters
alpha = 0.5  # Complementary filter coefficient
pitchGyro = 0.0  # Initial pitch based on gyroscope integration
rollGyro = 0.0   # Initial roll based on gyroscope integration

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

def complementary_filter_update(accel, gyro, dt):
    global pitchGyro, rollGyro
    accel = normalize(accel)

    # Integrate the gyroscope data
    pitchGyro += gyro[0] * dt
    rollGyro += gyro[1] * dt

    # Calculate pitch and roll from the accelerometer
    pitchAcc = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
    rollAcc = np.arctan2(accel[1], accel[2])

    # Apply the complementary filter
    pitch = alpha * pitchGyro + (1 - alpha) * pitchAcc
    roll = alpha * rollGyro + (1 - alpha) * rollAcc

    return pitch, roll

def quaternion_to_euler(q):
    r = R.from_quat(q)
    return r.as_euler('xyz', degrees=True)

def receive_udp_data():
    global last_print_time
    UDP_IP = "0.0.0.0"
    UDP_PORT = 12346

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow the socket to reuse the address
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP packets on port {UDP_PORT}...")

    prev_time = None

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

                timestamp = time.time()
                if prev_time is None:
                    prev_time = timestamp

                dt = timestamp - prev_time
                prev_time = timestamp

                # Update the pitch and roll using the complementary filter
                pitch, roll = complementary_filter_update(accel, gyro, dt)

                # Convert pitch and roll to degrees
                pitch_deg = np.degrees(pitch)
                roll_deg = np.degrees(roll)

                current_time = time.time()
                if current_time - last_print_time >= 0.2:  # 100ms interval
                    print(f"Calibrated Accel: {accel}, Calibrated Gyro: {gyro}")
                    print(f"Pitch: {pitch_deg:.2f} degrees, Roll: {roll_deg:.2f} degrees")
                    last_print_time = current_time
            else:
                if part:
                    print(f"Received packet of incorrect size: {len(part)} bytes")

if __name__ == "__main__":
    acc_calibration_file = "./main server GUI/calib_data/test_imu_acc.calib"
    gyro_calibration_file = "./main server GUI/calib_data/test_imu_gyro.calib"

    acc_misalignment, acc_scale, acc_bias = load_calibration(acc_calibration_file)
    gyro_misalignment, gyro_scale, gyro_bias = load_calibration(gyro_calibration_file)
    
    receive_udp_data()
