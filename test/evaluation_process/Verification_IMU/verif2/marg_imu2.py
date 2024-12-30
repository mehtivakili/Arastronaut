import numpy as np
import socket
import struct
import threading
import queue
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from ahrs.filters import Mahony  # or Madgwick
import time
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


# Function to apply calibration to accelerometer and gyroscope data
def apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias):
    # Apply calibration to accelerometer data
    accel_corrected = np.dot(acc_misalignment, accel - acc_bias)
    accel_calibrated = np.dot(acc_scale, accel_corrected)

    # Apply calibration to gyroscope data
    gyro_corrected = np.dot(gyro_misalignment, gyro - gyro_bias)
    gyro_calibrated = np.dot(gyro_scale, gyro_corrected)

    return accel_calibrated, gyro_calibrated


# Function to apply magnetometer calibration (offset and scale)
def apply_mag_calibration(mag, mag_offsets, mag_scales):
    # Apply the calibration (hard iron correction and scaling)
    mag_corrected = (mag - mag_offsets) / mag_scales
    return mag_corrected


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


# Apply rotation to the cube using quaternion
def rotate_cube(vertices, quaternion):
    rotation = R.from_quat(quaternion)  # Create a Rotation object from the quaternion
    rotated_vertices = rotation.apply(vertices)  # Apply the rotation to the vertices
    return rotated_vertices


# Function to process data in a separate thread using AHRS (Mahony or Madgwick filter)
def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales):
    # Initialize the AHRS filter (Mahony or Madgwick)
    ahrs_filter = Mahony(frequency=200)  # You can also use Madgwick() here
    q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion (identity)
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

                        # Process data (assumed 1 int64 timestamp and 9 floats for IMU data)
                        values = struct.unpack('<q9f', part)

                        timestamp_ns = values[0]
                        accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]

                        # Prepare data arrays
                        accel = np.array([accelX, accelY, accelZ])
                        gyro = np.array([gyroX, gyroY, gyroZ])
                        mag = np.array([magX, magY, magZ])

                        # Apply calibration
                        accel, gyro = apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias)
                        mag_calibrated = apply_mag_calibration(mag, mag_offsets, mag_scales)

                        # Update quaternion using the AHRS filter
                        q = ahrs_filter.updateMARG(q, gyr=gyro, acc=accel, mag=mag_calibrated)
                        print(q)
                        # Send quaternion to the visualization thread
                        data_queue.put(q)

        except socket.timeout:
            print("Waiting for data...")
        except KeyboardInterrupt:
            print("Terminating data thread...")
            break


# Visualization function (runs in the main thread)
def visualization_thread(data_queue):
    # Initialize plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])

    # Define cube size
    cube_size = 1.0
    cube_vertices = create_cube(cube_size)

    while True:
        try:
            quaternion = data_queue.get(timeout=1)  # Get quaternion from the data thread

            # # Rotate cube vertices
            # rotated_vertices = rotate_cube(cube_vertices, quaternion)
            # faces = get_cube_faces(rotated_vertices)

            # # Clear the plot
            # ax.clear()

            # Plot the cube
            # ax.add_collection3d(Poly3DCollection(faces, facecolors='cyan', linewidths=1, edgecolors='r', alpha=.25))

            # # Set the axes limits
            # ax.set_xlim([-cube_size, cube_size])
            # ax.set_ylim([-cube_size, cube_size])
            # ax.set_zlim([-cube_size, cube_size])

            # # Set labels and title
            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')
            # ax.set_zlabel('Z')
            # ax.set_title(f'Cube Rotation - Quaternion: {quaternion}')

            # plt.pause(0.01)  # Pause to update the plot

        except queue.Empty:
            continue


# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# Load calibration data for accelerometer and gyroscope
acc_misalignment, acc_scale, acc_bias = load_calibration('./main server GUI/calib_data/test_imu_acc6G.calib')
gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./main server GUI/calib_data/test_imu_gyro.calib')

# Example calibration offsets and scales for magnetometer
mag_offsets = np.array([585.0, 310.0, 40.0])  # Offsets for X, Y, Z axes
mag_scales = np.array([2164.0, 2450.0, 2356.0])  # Scales for X, Y, Z axes

# Create a queue to pass data between threads
data_queue = queue.Queue()

# Start data collection thread
data_thread = threading.Thread(target=data_thread, args=(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales))
data_thread.daemon = True
data_thread.start()

# Start visualization in the main thread
visualization_thread(data_queue)
