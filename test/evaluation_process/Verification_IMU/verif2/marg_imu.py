import numpy as np
import socket
import struct
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

# Function to load IMU calibration data from files
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
# Example calibration offsets and scales for magnetometer
mag_offsets = np.array([537.0, 214.0, 55.0])  # Offsets for X, Y, Z axes
mag_scales = np.array([2510.0, 2172.0, 2356.0])  # Scales for X, Y, Z axes

# Load calibration data for accelerometer and gyroscope
acc_misalignment, acc_scale, acc_bias = load_calibration('./main server GUI/calib_data/test_imu_acc.calib')
gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./main server GUI/calib_data/test_imu_gyro.calib')

# Function to apply calibration to accelerometer and gyroscope data
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

print(f"Listening on {UDP_IP}:{UDP_PORT}")

# Initialize the Madgwick MARG filter
madgwick = Madgwick()

# Initialize the quaternion (unit quaternion, representing no initial rotation)
q = np.array([1.0, 0.0, 0.0, 0.0])


# Main loop for receiving and processing data
rate = 0

while True:
    try:
        # Receive data from UDP
        data, addr = sock.recvfrom(4096)

        # Check for specific header in the data
        check = "img/"
        check_encoded = check.encode()
        parts = data.split(check_encoded)

        # Process each part of the received data
        for part in parts:
            if len(part) == 44:  # 64-bit timestamp + 9 floats (9 sensor readings)
                rate += 1
                if rate == 40:  # Process every 10th packet (adjust rate as needed)
                    rate = 0

                    # Unpack the data (1 int64 timestamp and 9 floats for IMU data)
                    values = struct.unpack('<q9f', part)
                    timestamp_ns = values[0]
                    timestamp_s = timestamp_ns / 1e9  # Convert nanoseconds to seconds
                    accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]

                    # Convert sensor data into numpy arrays
                    accel = np.array([accelX, accelY, accelZ])
                    gyro = np.array([gyroX, gyroY, gyroZ])
                    mag = np.array([magX, magY, magZ])

                    # Apply calibration to accelerometer and gyroscope data
                    accel, gyro = apply_calibration(accel, gyro)

                    # Calibrate magnetometer data
                    mag = calibrate_data(mag, mag_offsets, mag_scales)

                    # Apply the Madgwick MARG filter
                    quaternion = madgwick.updateMARG(q=q, acc=accel, gyr=gyro, mag=mag)

                    # Convert quaternion to Euler angles (roll, pitch, yaw)
                    rotation = R.from_quat(quaternion)
                    euler_angles = rotation.as_euler('xyz', degrees=True)

                    if rate % 10 == 0:  # Print every 10th iteration
                        print(f"Euler Angles (roll, pitch, yaw): {euler_angles}")

                    # # Print calibrated sensor data and orientation
                    # print(f"Timestamp: {timestamp_s}s")
                    # print(f"Calibrated Accelerometer: {accel}")
                    # print(f"Calibrated Gyroscope: {gyro}")
                    # print(f"Calibrated Magnetometer: {mag}")
                    # print(f"Euler Angles (roll, pitch, yaw): {euler_angles}")

    except socket.timeout:
        print("Waiting for data...")
    except KeyboardInterrupt:
        print("Terminating program...")
        break
