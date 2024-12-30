import numpy as np
import socket
import struct
import threading
import queue
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
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

# Function to apply calibration to accelerometer and gyroscope data
def apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias):
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

# Complementary filter function to combine accelerometer, gyroscope, and magnetometer data
def complementary_filter(accel, gyro, mag, dt, previous_roll, previous_pitch, previous_yaw, alpha=0.98):
    # Roll and pitch from accelerometer
    acc_roll = np.arctan2(accel[1], np.sqrt(accel[0] ** 2 + accel[2] ** 2)) * 180 / np.pi
    acc_pitch = np.arctan2(-accel[0], np.sqrt(accel[1] ** 2 + accel[2] ** 2)) * 180 / np.pi
    
    # Integrate gyroscope data (angular velocity) to get change in angles
    gyro_roll_rate = gyro[0]
    gyro_pitch_rate = gyro[1]
    gyro_yaw_rate = gyro[2]

    # Update roll and pitch with complementary filter
    roll = alpha * (previous_roll + gyro_roll_rate * dt) + (1 - alpha) * acc_roll
    pitch = alpha * (previous_pitch + gyro_pitch_rate * dt) + (1 - alpha) * acc_pitch

    # Yaw from magnetometer
    mag_yaw = np.arctan2(mag[1], mag[0]) * 180 / np.pi
    
    # Apply complementary filter to yaw
    yaw = alpha * (previous_yaw + gyro_yaw_rate * dt) + (1 - alpha) * mag_yaw
    
    # Unwrap yaw to prevent sudden jumps
    yaw = unwrap_angle(previous_yaw, yaw)

    return roll, pitch, yaw


def unwrap_angle(previous_angle, current_angle):
    """Unwrap angles to avoid discontinuity when crossing -180 to 180 degrees."""
    delta_angle = current_angle - previous_angle
    
    # Adjust the angle difference if it crosses the boundary
    if delta_angle > 180:
        current_angle -= 360
    elif delta_angle < -180:
        current_angle += 360
    
    return current_angle


# Function to process data in a separate thread
def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales):
    previous_roll = 0.0
    previous_pitch = 0.0
    previous_yaw = 0.0
    previous_timestamp_s = 0.0
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
                    if rate == 20:  # Process every 10th packet (adjust rate as needed)
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
                        print(mag)

                        # Apply calibration to accelerometer and gyroscope data
                        accel, gyro = apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias)

                        # Calibrate magnetometer data
                        mag = calibrate_data(mag, mag_offsets, mag_scales)

                        # Calculate time difference (dt) between measurements
                        dt = timestamp_s - previous_timestamp_s
                        previous_timestamp_s = timestamp_s

                        # Apply complementary filter to get roll, pitch, and yaw
                        roll, pitch, yaw = complementary_filter(accel, gyro, mag, dt, previous_roll, previous_pitch, previous_yaw, alpha=0.05)

                        # Update previous roll, pitch, yaw
                        previous_roll, previous_pitch, previous_yaw = roll, pitch, yaw

                        # Send roll, pitch, yaw to the visualization thread
                        data_queue.put((roll, pitch, yaw))

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
            # Get data from the queue
            roll, pitch, yaw = data_queue.get()

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

        except queue.Empty:
            pass

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
