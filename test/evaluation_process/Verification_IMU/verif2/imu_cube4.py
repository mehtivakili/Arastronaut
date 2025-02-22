import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
import socket
import struct
import time

# Covariance matrices for accelerometer, gyroscope, and magnetometer
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

# Magnetometer covariance matrix
mag_cov_matrix = np.array([
    [0.001, 0.0001, 0.0001],
    [0.0001, 0.002, 0.0002],
    [0.0001, 0.0002, 0.0015]
])

# Magnetometer calibration offsets and scales
mag_offsets = np.array([516.0, 322.0, 141.0])  # Example offsets for X, Y, Z axes
mag_scales = np.array([2499.0, 2417.0, 2575.0])  # Example scales for X, Y, Z axes

# Initial Kalman filter states
acc_P = np.eye(3)  # Error covariance matrix for accelerometer
gyro_P = np.eye(3)  # Error covariance matrix for gyroscope
mag_P = np.eye(3)  # Error covariance matrix for magnetometer

# Measurement noise covariance
R_acc = np.eye(3) * 0.01
R_gyro = np.eye(3) * 0.01
R_mag = mag_cov_matrix  # Use the magnetometer covariance matrix here

# Initial states
acc_x = np.zeros((3, 1))
gyro_x = np.zeros((3, 1))
mag_x = np.zeros((3, 1))

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

# Load calibration data for accelerometer and gyroscope
acc_misalignment, acc_scale, acc_bias = load_calibration('./main server GUI/calib_data/test_imu_acc.calib')
gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./main server GUI/calib_data/test_imu_gyro.calib')

# Function to apply calibration to accelerometer and gyroscope
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

# Function to calculate roll, pitch, and yaw using accelerometer, gyroscope, and magnetometer
def calculate_roll_pitch_yaw(accel, gyro, mag, dt, prev_yaw):
    # Roll (rotation around X-axis)
    roll = np.arctan2(accel[1], accel[2])

    # Pitch (rotation around Y-axis)
    pitch = np.arctan2(-accel[0], np.sqrt(accel[1] ** 2 + accel[2] ** 2))

    # Yaw (rotation around Z-axis) using magnetometer
    mag_x = mag[0] * np.cos(pitch) + mag[2] * np.sin(pitch)
    mag_y = mag[0] * np.sin(roll) * np.sin(pitch) + mag[1] * np.cos(roll) - mag[2] * np.sin(roll) * np.cos(pitch)
    yaw = np.arctan2(-mag_y, mag_x)

    # Convert radians to degrees
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)

    return roll, pitch, yaw

# Kalman filter function
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

# Create a cube with vertices
def create_cube(size=1.0):
    vertices = np.array([
        [-size, -size, -size],
        [size, -size, -size],
        [size, size, -size],
        [-size, size, -size],
        [-size, -size, size],
        [size, -size, size],
        [size, size, size],
        [-size, size, size]
    ])
    return vertices

# Define the faces of the cube
def get_cube_faces(vertices):
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],
        [vertices[4], vertices[5], vertices[6], vertices[7]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[0], vertices[3], vertices[7], vertices[4]],
        [vertices[1], vertices[2], vertices[6], vertices[5]]
    ]
    return faces

# Apply rotation to the cube
def rotate_cube(vertices, roll, pitch, yaw):
    rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    rotated_vertices = rotation.apply(vertices)
    return rotated_vertices

# Initialize plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1, 1, 1])

# Define cube size
cube_size = 1.0
cube_vertices = create_cube(cube_size)

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# Start time
prev_yaw = 0.0  # Initialize previous yaw
rate = 0

# Main loop to update the cube
while True:
    data, addr = sock.recvfrom(4096)
    check = "img/"
    check_encoded = check.encode()
    parts = data.split(check_encoded)

    for part in parts:
        if len(part) == 44:  # 64-bit timestamp + 9 floats
            rate = rate + 1
            if rate == 30:
                values = struct.unpack('<q9f', part)
                timestamp_ns = values[0]
                timestamp_s = timestamp_ns / 1e9  # Convert nanoseconds to seconds
                accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]

                accel = np.array([accelX, accelY, accelZ])
                gyro = np.array([gyroX, gyroY, gyroZ])
                mag = np.array([magX, magY, magZ])

                # Apply calibration to accelerometer and gyroscope data
                accel, gyro = apply_calibration(accel, gyro)

                # Calibrate magnetometer data
                mag = calibrate_data(mag, mag_offsets, mag_scales)

                # Apply Kalman filter to accelerometer data
                acc_x, acc_P = kalman_filter(accel, acc_x, acc_P, acc_cov_matrix, R_acc)

                # Apply Kalman filter to gyroscope data
                gyro_x, gyro_P = kalman_filter(gyro, gyro_x, gyro_P, gyro_cov_matrix, R_gyro)

                # Apply Kalman filter to magnetometer data
                mag_x, mag_P = kalman_filter(mag, mag_x, mag_P, mag_cov_matrix, R_mag)

                # Calculate time difference
                current_time = time.time()
                dt = current_time - timestamp_s

                # Calculate roll, pitch, and yaw using the filtered data
                roll, pitch, yaw = calculate_roll_pitch_yaw(acc_x.flatten(), gyro_x.flatten(), mag_x.flatten(), dt, prev_yaw)
                prev_yaw = yaw  # Update the previous yaw

                # Rotate cube vertices
                rotated_vertices = rotate_cube(cube_vertices, roll, pitch, yaw)
                faces = get_cube_faces(rotated_vertices)

                # Clear the plot
                ax.clear()

                # Plot the cube
                ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

                # Set the axes limits
                ax.set_xlim([-cube_size, cube_size])
                ax.set_ylim([-cube_size, cube_size])
                ax.set_zlim([-cube_size, cube_size])

                # Set labels and title
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title(f'Cube Rotation - Roll: {roll:.2f}, Pitch: {pitch:.2f}, Yaw: {yaw:.2f}')

                plt.pause(0.01)  # Pause to update the plot
                rate = 0

plt.show()
