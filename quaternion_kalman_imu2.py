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

# Apply rotation to the cube using quaternion
def rotate_cube(vertices, quaternion):
    rotation = R.from_quat(quaternion)
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

# Function to normalize a quaternion
def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return q / norm

def normalize_yaw(yaw):
    while yaw > 180:
        yaw -= 360
    while yaw < -180:
        yaw += 360
    return yaw


# Function to convert Euler angles (roll, pitch, yaw) to a quaternion
def euler_to_quaternion(roll, pitch, yaw):
    # Convert degrees to radians
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(normalize_yaw(yaw))

    # Compute quaternion components
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy

    # Create the quaternion array
    q = np.array([q_x, q_y, q_z, q_w])

    # Normalize the quaternion before returning it
    return normalize_quaternion(q)

def invert_3x3_matrix(m):
    det = (m[0, 0] * (m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) -
           m[0, 1] * (m[1, 0] * m[2, 2] - m[1, 2] * m[2, 0]) +
           m[0, 2] * (m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]))
    
    inv_det = 1.0 / det
    
    inv_m = np.array([
        [(m[1, 1] * m[2, 2] - m[1, 2] * m[2, 1]) * inv_det,
         (m[0, 2] * m[2, 1] - m[0, 1] * m[2, 2]) * inv_det,
         (m[0, 1] * m[1, 2] - m[0, 2] * m[1, 1]) * inv_det],
        
        [(m[1, 2] * m[2, 0] - m[1, 0] * m[2, 2]) * inv_det,
         (m[0, 0] * m[2, 2] - m[0, 2] * m[2, 0]) * inv_det,
         (m[0, 2] * m[1, 0] - m[0, 0] * m[1, 2]) * inv_det],
        
        [(m[1, 0] * m[2, 1] - m[1, 1] * m[2, 0]) * inv_det,
         (m[0, 1] * m[2, 0] - m[0, 0] * m[2, 1]) * inv_det,
         (m[0, 0] * m[1, 1] - m[0, 1] * m[1, 0]) * inv_det]
    ])
    
    return inv_m


# Kalman filter function to combine accelerometer, gyroscope, and magnetometer data
def kalman_filter_update(x, P, z, A, Q, R, H):
    """
    Kalman filter update step to update the state (roll, pitch, yaw)
    with new accelerometer and magnetometer measurements.
    """
    # Predict
    x_pred = A @ x
    P_pred = A @ P @ A.T + Q

    # Update (using accelerometer and magnetometer measurements)
    # Replace this:
    K = P_pred @ H.T @ np.linalg.inv(H @ P_pred @ H.T + R)

    # With this:
    S = H @ P_pred @ H.T + R  # Intermediate matrix
    S_inv = invert_3x3_matrix(S)  # Use efficient inversion
    K = P_pred @ H.T @ S_inv    
    x = x_pred + K @ (z - H @ x_pred)
    P = (np.eye(len(K)) - K @ H) @ P_pred

    return x, P

# Function to process data in a separate thread
def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales):
    # Initial state [roll, pitch, yaw]
    x = np.zeros(3)
    
    # State covariance matrix
    P = np.eye(3, dtype=np.float32)

    # Process noise covariance (from gyroscope covariance)
    Q = np.array([[1.810157e-06, -1.308623e-07, -8.375072e-08],
                  [-1.308623e-07,  2.236266e-06,  1.143754e-07],
                  [-8.375072e-08,  1.143754e-07,  8.101775e-07]], dtype=np.float32)

    # Measurement noise covariance (from accelerometer and magnetometer covariance)
    R = np.array([[0.000716,  0.000058, -0.000001],
                  [0.000058,  0.000830, -0.000028],
                  [-0.000001, -0.000028,  0.000788]], dtype=np.float32)

    # State transition matrix
    A = np.eye(3)

    # Measurement matrix (maps state to measurements)
    H = np.eye(3)

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
                    if rate == 40:  # Process every 10th packet (adjust rate as needed)
                        rate = 0

                        # Unpack the data (1 int64 timestamp and 9 floats for IMU data)
                        values = struct.unpack('<q9f', part)
                        timestamp_ns = values[0]
                        timestamp_s = timestamp_ns / 1e9  # Convert nanoseconds to seconds
                        accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]

                        # Convert sensor data into numpy arrays
                        accel = np.array([accelX, accelY, accelZ], dtype=np.float32)
                        gyro = np.array([gyroX, gyroY, gyroZ], dtype=np.float32)
                        mag = np.array([magX, magY, magZ], dtype=np.float32)

                        # Apply calibration to accelerometer and gyroscope data
                        accel, gyro = apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias)

                        # Calculate time difference (dt) between measurements
                        dt = timestamp_s - previous_timestamp_s
                        previous_timestamp_s = timestamp_s

                        # Calculate yaw from magnetometer
                        yaw_mag = np.arctan2(mag[1], mag[0]) * 180 / np.pi

                        # Use accelerometer data for roll, pitch, and magnetometer for yaw
                        roll_acc = np.arctan2(accel[1], np.sqrt(accel[0] ** 2 + accel[2] ** 2)) * 180 / np.pi
                        pitch_acc = np.arctan2(-accel[0], np.sqrt(accel[1] ** 2 + accel[2] ** 2)) * 180 / np.pi
                        z = np.array([roll_acc, pitch_acc, yaw_mag])

                        # Apply Kalman filter to get roll, pitch, and yaw
                        x, P = kalman_filter_update(x, P, z, A, Q, R, H)

                        # Update the state with gyro data (simple integration)
                        x = x + gyro * dt  # Integrate gyro data to update state

                        roll, pitch, yaw = x
                        print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

                        # Convert Euler angles to quaternion
                        quaternion = euler_to_quaternion(roll, pitch, yaw)

                        # Send quaternion to the visualization thread
                        data_queue.put(quaternion)

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
            quaternion = data_queue.get()

            print(f"Quaternion (w, x, y, z): {quaternion}")

            # Rotate cube vertices
            rotated_vertices = rotate_cube(cube_vertices, quaternion)
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
            ax.set_title(f'Cube Rotation - Quaternion: {quaternion}')

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
