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
# Function to apply calibration to accelerometer and gyroscope data
def apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias):
    # Apply calibration to accelerometer data
    accel_corrected = np.dot(acc_misalignment, accel - acc_bias)
    accel_calibrated = np.dot(acc_scale, accel_corrected)
    
    # Apply calibration to gyroscope data
    gyro_corrected = np.dot(gyro_misalignment, gyro - gyro_bias)
    gyro_calibrated = np.dot(gyro_scale, gyro_corrected)
    
    return accel_calibrated, gyro_calibrated


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

# Function to normalize a quaternion
def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    return q / norm

# Low-pass filter for gyroscope data
def low_pass_filter(gyro, previous_gyro, alpha=0.98):
    return alpha * previous_gyro + (1 - alpha) * gyro

# Update the quaternion with low-pass filtered gyro data
def update_quaternion_with_filtered_gyro(quaternion, gyro, previous_gyro, dt, alpha=0.98):
    # Apply the low-pass filter to the gyroscope data
    gyro_filtered = low_pass_filter(gyro, previous_gyro, alpha)
    
    # Convert the filtered gyro angular velocity to a quaternion delta
    delta_q = R.from_rotvec(gyro_filtered * dt).as_quat()
    
    # Update the quaternion by multiplying with the delta quaternion
    quaternion_updated = R.from_quat(quaternion) * R.from_quat(delta_q)
    return quaternion_updated.as_quat(), gyro_filtered

# Function to correct quaternion using accelerometer and magnetometer
# Correct roll and pitch using accelerometer and yaw using magnetometer
def correct_quaternion_with_accel_mag(accel, mag):
    # Convert accelerometer data to quaternion (roll and pitch)
    accel_quat = R.from_euler('xyz', [
        np.arctan2(accel[1], accel[2]), 
        np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2)), 
        0  # No yaw correction from accel
    ]).as_quat()
    
    # Calculate yaw from magnetometer data
    mag_yaw = np.arctan2(mag[1], mag[0]) * np.pi / 180
    mag_quat = R.from_euler('z', [mag_yaw]).as_quat()  # Only yaw correction

    # Combine the two quaternions (accel for roll/pitch and mag for yaw)
    corrected_quat = R.from_quat(accel_quat) * R.from_quat(mag_quat)
    
    # Normalize the result
    corrected_quat = corrected_quat.as_quat()
    corrected_quat = corrected_quat / np.linalg.norm(corrected_quat)  # Normalize

    return corrected_quat


# Apply rotation to the cube using quaternion
def rotate_cube(vertices, quaternion):
    rotation = R.from_quat(quaternion)  # Create a Rotation object from the quaternion
    rotated_vertices = rotation.apply(vertices)  # Apply the rotation to the vertices
    return rotated_vertices

def low_pass_filter_gyro(gyro, previous_gyro, alpha=0.98):
    return alpha * previous_gyro + (1 - alpha) * gyro

# Kalman filter function to update the quaternion
def kalman_filter_update_quaternion(q, P, z_quat, A, Q, R_cov, H):
    """
    Kalman filter update for quaternion state.
    q: quaternion state (4D)
    P: state covariance matrix (4x4)
    z_quat: quaternion from measurements (4D)
    A: state transition matrix (4x4)
    Q: process noise covariance (4x4)
    R_cov: measurement noise covariance (4x4)
    H: measurement matrix (4x4)
    """
    # Predict step
    q_pred = A @ q  # Predict quaternion state
    P_pred = A @ P @ A.T + Q  # Predict covariance

    # Reshape z_quat to match the shape of q_pred
    z_quat = z_quat.squeeze()  # Ensure z_quat has shape (4,)
    
    # Update step using quaternion measurements
    S = H @ P_pred @ H.T + R_cov  # Calculate Kalman gain
    K = P_pred @ H.T @ np.linalg.inv(S)
    
    # Ensure z_quat, q_pred, and H @ q_pred are compatible (4D)
    print(f"q_pred shape: {q_pred.shape}")
    print(f"K shape: {K.shape}")
    print(f"z_quat shape: {z_quat.shape}")
    print(f"H @ q_pred shape: {(H @ q_pred).shape}")

    # Update the quaternion
    q = q_pred + K @ (z_quat - H @ q_pred)
    P = (np.eye(len(K)) - K @ H) @ P_pred

    return normalize_quaternion(q), P

# Apply Kalman filter and fuse with corrected quaternion
# Kalman filter and gyro correction function
def apply_kalman_with_gyro_correction(q, P, gyro, accel, mag, dt, previous_gyro, A, Q, R_cov, H):
    # Apply low-pass filter to gyroscope data to reduce noise
    gyro_filtered = low_pass_filter_gyro(gyro, previous_gyro)

    # Update quaternion with gyro data (gyro integration)
    q_pred, gyro_filtered = update_quaternion_with_filtered_gyro(q, gyro_filtered, previous_gyro, dt)

    # Correct quaternion using accelerometer and magnetometer
    corrected_quat = correct_quaternion_with_accel_mag(accel, mag)

    # Calculate the quaternion error (q_err = q_corrected * q_pred_inverse)
    q_pred_inv = R.from_quat(q_pred).inv()
    q_err = R.from_quat(corrected_quat) * q_pred_inv
    q_err = q_err.as_rotvec()  # Convert to a 3D rotation vector (small angle approximation)

    # Ensure q_err is 1D
    q_err = q_err.flatten()  # Shape: (3,)

    # Kalman filter prediction step
    P_pred = A @ P @ A.T + Q  # A, P, Q should all be 4x4 for quaternion state

    # Measurement update
    S = H[:3, :3] @ P_pred[:3, :3] @ H[:3, :3].T + R_cov[:3, :3]  # 3x3 for rotation vector correction
    K = P_pred[:3, :3] @ H[:3, :3].T @ np.linalg.inv(S)  # Kalman gain is 3x3

    # Apply Kalman gain to the 3D rotation vector
    q_delta = K @ q_err  # Apply correction in the rotation vector space (q_err is 3D)

    # Apply the quaternion correction (q = q_pred * q_delta)
    q_delta_rot = R.from_rotvec(q_delta).as_quat()  # Convert small-angle correction to quaternion
    q = R.from_quat(q_pred) * R.from_quat(q_delta_rot)
    q = q.as_quat()

    # Update state covariance matrix
    P[:3, :3] = (np.eye(3) - K @ H[:3, :3]) @ P_pred[:3, :3]  # Only update the 3x3 rotational part of P

    # Normalize quaternion to ensure valid rotation
    q = q / np.linalg.norm(q)

    return q, P, gyro_filtered






# Function to process data in a separate thread
def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales):
    previous_gyro = np.zeros(3)
    previous_timestamp_s = 0.0
    q = np.array([1.0, 0.0, 0.0, 0.0])  # Initial quaternion
    # State covariance matrix (4x4)
    P = np.eye(4) * 1e-2
    # Define matrices A, Q, R_cov, H for Kalman filter
    A = np.eye(4)  # State transition matrix
    Q = np.eye(4) * 1e-5  # Process noise covariance
    R_cov = np.eye(4) * 1e-2  # Measurement noise covariance
    H = np.eye(4)  # Measurement matrix

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

                        # Convert sensor data to numpy arrays
                        accel = np.array([accelX, accelY, accelZ])
                        gyro = np.array([gyroX, gyroY, gyroZ])
                        mag = np.array([magX, magY, magZ])

                        # Apply calibration to accelerometer and gyroscope data
                        accel, gyro = apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias)

                        # Calculate time difference (dt) between measurements
                        dt = timestamp_s - previous_timestamp_s
                        previous_timestamp_s = timestamp_s

                        # Apply Kalman filter with gyroscope correction
                        q, P, previous_gyro = apply_kalman_with_gyro_correction(
                            q, P, gyro, accel, mag, dt, previous_gyro, A, Q, R_cov, H
                        )

                        print(f"Quaternion: {q}")

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
            # Get data from the queue
            quaternion = data_queue.get()

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
