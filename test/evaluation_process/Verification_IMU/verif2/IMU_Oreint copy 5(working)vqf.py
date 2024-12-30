import numpy as np
import socket
import struct
import threading
import queue
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from vqf import VQF
import time
import csv
from scipy.spatial.transform import Rotation as R

# Constants
MAG_LOWER_BOUND = -100000
MAG_UPPER_BOUND = 100000

# Helper functions
def filter_magnetometer(mag, lower_bound, upper_bound):
    """Filter magnetometer data based on threshold."""
    return np.all((mag >= lower_bound) & (mag <= upper_bound))

def load_calibration(file):
    """Load IMU calibration data from files."""
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

def create_cube(size=1.0):
    """Create a cube with vertices."""
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

def get_cube_faces(vertices):
    """Define the faces of the cube."""
    faces = [
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom face
        [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top face
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front face
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back face
        [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left face
        [vertices[1], vertices[2], vertices[6], vertices[5]]   # Right face
    ]
    return faces

def rotate_cube_with_quaternion(vertices, q):
    """Rotate cube vertices using a quaternion."""
    # Convert quaternion to Rotation object
    rotation = R.from_quat([q[1], q[2], q[3], q[0]])  # [x, y, z, w] format for scipy
    # Apply the quaternion rotation to each vertex
    rotated_vertices = rotation.apply(vertices)
    return rotated_vertices

def apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias):
    """Apply calibration to accelerometer and gyroscope data."""
    # Apply calibration to accelerometer data
    accel_corrected = np.dot(acc_misalignment, accel - acc_bias)
    accel_calibrated = np.dot(acc_scale, accel_corrected)
    # Apply calibration to gyroscope data
    gyro_corrected = np.dot(gyro_misalignment, gyro - gyro_bias)
    gyro_calibrated = np.dot(gyro_scale, gyro_corrected)
    return accel_calibrated, gyro_calibrated

def calibrate_magnetometer(data, offsets, scales, scale_factors=None):
    """Calibrate and scale magnetometer data."""
    # Subtract the hard iron correction (offsets)
    data_corrected = data - offsets
    # Apply the soft iron correction matrix (scales)
    calibrated_data = np.dot(data_corrected, scales)
    # Apply provided scaling factors if available
    if scale_factors is not None:
        calibrated_data /= scale_factors
    return calibrated_data

def normalize_quaternion(q):
    """Normalize a quaternion to unit length."""
    norm = np.linalg.norm(q)
    if norm == 0:
        return q  # Avoid division by zero
    return q / norm

# Visualization function
def visualization_thread(data_queue):
    # Initialize plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])
    plt.ion()  # Enable interactive mode
    fig.show()

    # Define cube size and vertices
    cube_size = 1.0
    cube_vertices = create_cube(cube_size)
    face_colors = ['blue', 'red', 'yellow', 'blue', 'red', 'yellow']

    # Open CSV file to save data
    with open('dataset-screw3.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write the header
        csvwriter.writerow(['timestamp_ns', 'angle_axis'])
        while True:
            try:
                # Get quaternion from the queue
                q = data_queue.get(timeout=0.01)

                # Rotate cube vertices
                rotated_vertices = rotate_cube_with_quaternion(cube_vertices, q)
                faces = get_cube_faces(rotated_vertices)

                # Clear the plot
                ax.clear()

                # Plot the cube
                ax.add_collection3d(Poly3DCollection(faces, facecolors=face_colors, linewidths=1, edgecolors='r', alpha=.25))

                # Set the axes limits
                ax.set_xlim([-cube_size*2, cube_size*2])
                ax.set_ylim([-cube_size*2, cube_size*2])
                ax.set_zlim([-cube_size*2, cube_size*2])

                # Refresh the plot
                plt.draw()
                plt.pause(0.01)  # Pause to update the plot
            except queue.Empty:
                print("Queue is empty, no data received")  # Debug: Check if the queue is empty
            except KeyboardInterrupt:
                print("Terminating visualization thread...")
                break

# Data processing function
def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias,
                gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales, scale_factors):
    previous_timestamp_s = 0.0
    i = 0
    # Initialize VQF filter
    vqf_filter = VQF(0.005)  # Example with 200 Hz sampling rate

    while True:
        try:
            # Receive data from UDP
            data, addr = sock.recvfrom(4096)
            # Check for specific header in the data
            check_encoded = "img/".encode()
            parts = data.split(check_encoded)
            for part in parts:
                if len(part) == 44:  # 64-bit timestamp + 9 floats (9 sensor readings)
                    # Unpack the data
                    values = struct.unpack('<q9f', part)
                    timestamp_ns = values[0]
                    timestamp_s = timestamp_ns / 1e9  # Convert nanoseconds to seconds
                    accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]
                    # Convert sensor data into numpy arrays
                    accel = np.array([-accelX, -accelY, -accelZ])
                    gyro = np.array([gyroX, gyroY, gyroZ])
                    mag = np.array([magX, magY, magZ])

                    # Filter the magnetometer data
                    if not filter_magnetometer(mag, MAG_LOWER_BOUND, MAG_UPPER_BOUND):
                        continue

                    # Apply calibration to accelerometer and gyroscope data
                    accel, gyro = apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias,
                                                    gyro_misalignment, gyro_scale, gyro_bias)
                    # Calibrate magnetometer data
                    mag = calibrate_magnetometer(mag, mag_offsets, mag_scales, scale_factors=scale_factors)

                    # Update the VQF filter with new sensor data
                    vqf_filter.updateGyr(gyro)
                    vqf_filter.updateAcc(accel)
                    vqf_filter.updateMag(mag)

                    # Get the updated quaternion
                    q = vqf_filter.getQuat9D()
                    print(q)
                    if q is not None:
                        i = i + 1
                        # Put the quaternion into the queue for visualization
                        if i == 30:
                            data_queue.put(q)
                            i = 0
        except socket.timeout:
            print("Waiting for data...")
        except KeyboardInterrupt:
            print("Terminating data thread...")
            break

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# Load calibration data for accelerometer and gyroscope
acc_misalignment, acc_scale, acc_bias = load_calibration('./../Arastronaut/calib_data/mpu_params2/test_imu_acc.calib')
gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./../Arastronaut/calib_data/mpu_params2/test_imu_gyro.calib')

# Define magnetometer calibration parameters
# mag_offsets = np.array([  -24.1425 , 339.4113, -191.2184])  # Hard iron correction bias
# mag_scales = np.array([[  0.9224,   -0.0167,    0.0341],
#                        [-0.0167,    1.0949,    0.0217],
#                        [ 0.0341,    0.0217,    0.9921]])


# # Define scale factors (from user input)
# scale_factors = np.array([227.5, 198, 204.5])  # Example scaling factors for X, Y, Z axes

# Define magnetometer calibration parameters
mag_offsets = np.array([ -2.1702, 61.6808, -32.3492])  # Hard iron correction bias
mag_scales = np.array([[ 0.9494,  -0.0026,    0.0446],
                       [ -0.0026,   1.0267,   0.0145],
                       [ 0.0446,    0.0145,   1.0380]])


# # Define scale factors (from user input)
scale_factors = np.array([70.851, 69.54, 62.782])  # Example scaling factors for X, Y, Z axes
# mag_offsets = np.array([ -8.0055, 64.5793, -33.4788])  # Hard iron correction bias
# mag_scales = np.array([[ 1,  0,    0],
#                        [ 0,   1,   0],
#                        [ 0,    0,   1]])


# # # Define scale factors (from user input)
# scale_factors = np.array([45.4262, 69.54, 44.4815])  # Example scaling factors for X, Y, Z axes


# Create a queue to pass data between threads
data_queue = queue.Queue()

# Start data collection thread  
data_thread_instance = threading.Thread(target=data_thread, args=(
    sock, data_queue, acc_misalignment, acc_scale, acc_bias,
    gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales, scale_factors))
data_thread_instance.daemon = True
data_thread_instance.start()

# Start visualization in the main thread
visualization_thread(data_queue)
