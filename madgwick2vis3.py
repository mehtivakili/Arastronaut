import numpy as np
import socket
import struct
import threading
import queue
from ahrs.filters import Madgwick
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
from ahrs.common import orientation
from pyquaternion import Quaternion

# Magnetometer data thresholds (adjust these based on your sensor's expected range)
MAG_LOWER_BOUND = -8000
MAG_UPPER_BOUND = 8000

# Function to filter magnetometer data based on threshold
def filter_magnetometer(mag, lower_bound, upper_bound):
    return np.all((mag >= lower_bound) & (mag <= upper_bound))

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
        [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom face
        [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top face
        [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front face
        [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back face
        [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left face
        [vertices[1], vertices[2], vertices[6], vertices[5]]   # Right face
    ]
    return faces

# Define colors for each face (6 faces total)
face_colors = ['blue', 'red', 'yellow', 'red', 'blue', 'yellow']

# # Apply rotation to the cube
# def rotate_cube(vertices, roll, pitch, yaw):
#     rotation = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
#     rotated_vertices = rotation.apply(vertices)
#     return rotated_vertices

# Function to rotate cube vertices using a quaternion
def rotate_cube_with_quaternion(vertices, q):
    # Convert quaternion to Rotation object
    rotation = R.from_quat([q[1], q[2], q[3], q[0]])  # [x, y, z, w] format for scipy

    # Apply the quaternion rotation to each vertex
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

import numpy as np

# Function to calibrate and scale magnetometer data
# def calibrate_data(data, offsets, scales, scale_factors=None):
#     """
#     Calibrates the magnetometer data using the provided offsets (hard iron correction)
#     and scales (soft iron correction matrix), and optionally applies user-provided scaling factors.

#     :param data: Raw magnetometer data (numpy array [magX, magY, magZ] or Nx3 matrix)
#     :param offsets: Hard iron correction bias (numpy array [offsetX, offsetY, offsetZ])
#     :param scales: Soft iron correction matrix (3x3 numpy array)
#     :param scale_factors: Optional scaling factors (numpy array [scaleX, scaleY, scaleZ]), default: None
#     :return: Calibrated (and optionally scaled) magnetometer data (numpy array [magX_cal, magY_cal, magZ_cal])
#     """
#     # Ensure data is a 2D array (Nx3) even if a single row is provided
#     # if data.ndim == 1:
#     #     data = data.reshape(1, -1)
    
#     # # Check for valid input shapes
#     # assert data.shape[1] == 3, "Data should have 3 columns representing X, Y, Z."
#     # assert offsets.shape == (3,), "Offsets should be a 1D array of size 3."
#     # assert scales.shape == (3, 3), "Scales should be a 3x3 matrix."
    
#     # Subtract the hard iron correction (offsets)
#     data_corrected = data - offsets

#     # Apply the soft iron correction matrix (scales)
#     calibrated_data = np.dot(data_corrected, scales)

#     # Apply provided scaling factors if available
#     if scale_factors is not None:
#         assert scale_factors.shape == (3,), "Scaling factors should be a 1D array of size 3."
#         calibrated_data /= scale_factors

#     return calibrated_data

# Function to compute the inverse (or conjugate) of a unit quaternion
def quaternion_inverse(q):
    w, x, y, z = q
    return np.array([w, -x, -y, -z])  # The inverse of a unit quaternion is its conjugate

# Function to multiply two quaternions
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w2 * w1 - x2 * x1 - y2 * y1 - z2 * z1
    x = w2 * x1 + x2 * w1 + y2 * z1 - z2 * y1
    y = w2 * y1 - x2 * z1 + y2 * w1 + z2 * x1
    z = w2 * z1 + x2 * y1 - y2 * x1 + z2 * w1

    return np.array([w, x, y, z])

# # Function to calibrate magnetometer data
def calibrate_data(data, offsets, scales):
    # Subtract the offsets and divide by the scales to normalize the data
    return (data - offsets) / scales


def euler2quat(roll, pitch, yaw):
    # Convert roll, pitch, and yaw (in radians) to a quaternion
    r = R.from_euler('xyz', [roll, pitch, yaw])
    return r.as_quat()  # Returns [x, y, z, w] quaternion

# def euler2quat(roll, pitch, yaw):
#     # Compute quaternion components from Euler angles
#     cy = np.cos(yaw * 0.5)
#     sy = np.sin(yaw * 0.5)
#     cp = np.cos(pitch * 0.5)
#     sp = np.sin(pitch * 0.5)
#     cr = np.cos(roll * 0.5)
#     sr = np.sin(roll * 0.5)

#     w = cr * cp * cy + sr * sp * sy
#     x = sr * cp * cy - cr * sp * sy
#     y = cr * sp * cy + sr * cp * sy
#     z = cr * cp * sy - sr * sp * cy

#     return np.array([w, x, y, z])  # Return quaternion [w, x, y, z]

# Initialize orientation using accelerometer and magnetometer
def initialize_orientation(accel, mag):
    accel_norm = accel / np.linalg.norm(accel)
    mag_norm = mag / np.linalg.norm(mag)
    
    # Calculate pitch and roll from accelerometer
    pitch = np.arctan2(accel_norm[1], accel_norm[2])
    roll = np.arctan2(-accel_norm[0], np.sqrt(accel_norm[1]**2 + accel_norm[2]**2))
    
    # Adjust magnetometer by pitch and roll to get correct yaw
    mag_x = mag_norm[0] * np.cos(pitch) + mag_norm[2] * np.sin(pitch)
    mag_y = mag_norm[0] * np.sin(roll) * np.sin(pitch) + mag_norm[1] * np.cos(roll) - mag_norm[2] * np.sin(roll) * np.cos(pitch)
    yaw = np.arctan2(-mag_y, mag_x)  # Yaw angle (heading) from magnetometer
    print(yaw/3.14 * 180, pitch/3.14 * 180, roll/3.14 * 180)
    # Convert to quaternion using scipy
    initial_quaternion = euler2quat(roll, pitch, yaw)
    return initial_quaternion

# Initialize the Madgwick filter
madgwick = Madgwick(frequency= 200, beta= 0.3)
# Initialize the quaternion (unit quaternion, representing no initial rotation)
q0=0

init_rot = True

# Function to process data in a separate thread
def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales):
    previous_timestamp_s = 0.0
    rate = 0
    rate2 = 0
    init_rot = True
    global q0
    # Initialize the quaternion (unit quaternion, representing no initial rotation)
    # q = np.array([1.0, 0.0, 0.0, 0.0])
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
                    if rate == 1:  # Process every 10th packet (adjust rate as needed)
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
                        # print(mag)
                        # Filter the magnetometer data
                        if not filter_magnetometer(mag, MAG_LOWER_BOUND, MAG_UPPER_BOUND):
                            # Skip the rest of the loop if magnetometer data is out of bounds
                            print(f"Magnetometer data out of bounds: {mag}, skipping...")
                            continue
                        # Apply calibration to accelerometer and gyroscope data
                        accel, gyro = apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias)

                        # Calibrate magnetometer data
                        mag = calibrate_data(mag, mag_offsets, scale_factors)
                        # mag = calibrate_data(mag, mag_offsets, mag_scales, scale_factors=scale_factors)

                        if (init_rot == True):
                            # Initialize the orientation using accelerometer and magnetometer
                            # initial_orientation = initialize_orientation(accel, mag)
                            # q = np.array([initial_orientation[3], initial_orientation[0], initial_orientation[1], initial_orientation[2]])
                            q = orientation.am2q(accel,mag)
                            q0=q.copy()
                            init_rot = False
                            # print(q)


                        # Calculate time difference (dt) between measurements
                        dt = timestamp_s - previous_timestamp_s
                        previous_timestamp_s = timestamp_s

                        # Update Madgwick filter with new sensor data
                        # If you have magnetometer data, use `updateMARG`; otherwise use `updateIMU`
                        q = madgwick.updateMARG(q=q, gyr=gyro, acc=accel, mag=mag)

                        if q is not None:
                            # Output the quaternion


                            # Optionally, convert quaternion to Euler angles (yaw, pitch, roll)
                            # Assuming q is [w, x, y, z] quaternion format
                            # pitch = np.arctan2(2.0 * (q[3] * q[2] + q[0] * q[1]), q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)
                            # roll = np.arcsin(-2.0 * (q[1] * q[3] - q[0] * q[2]))
                            # yaw = np.arctan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2)

                            # # Convert to degrees
                            # yaw = np.degrees(yaw)
                            # pitch = np.degrees(pitch)
                            # roll = np.degrees(roll)
                            yaw = np.arctan2(-mag[1], mag[0])  # Yaw angle (heading) from magnetometer
                            rate2 = rate2 + 1
                            if rate2 == 20:
                                # print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
                                # print(f"Quaternion: {q}")
                                # print(accel, gyro, mag)
                                print(yaw/3.14 * 180)
                                rate2 = 0

                                # Send roll, pitch, yaw to the visualization thread
                                # data_queue.put((roll, pitch, yaw))
                                data_queue.put(q)

        except socket.timeout:
            print("Waiting for data...")
        except KeyboardInterrupt:
            print("Terminating data thread...")
            break
    
        
def quaternion_to_euler(q):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        q: A quaternion in the form [qw, qx, qy, qz]
    
    Returns:
        A tuple of Euler angles (roll, pitch, yaw) in radians.
    """
    qw, qx, qy, qz = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx ** 2 + qy ** 2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.pi / 2 * np.sign(sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy ** 2 + qz ** 2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw
        

# Visualization function (runs in the main thread)
def visualization_thread(data_queue):
    # Initialize plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])

    # Define cube size
    cube_size = 1.0
    cube_vertices = create_cube(cube_size)
    
    global q0

    # Define colors for each face
    face_colors = ['blue', 'red', 'yellow', 'blue', 'red', 'yellow']
    rate2 = 0

    while True:
        try:
            # Get data from the queuec
            # roll, pitch, yaw = data_queue.get()
            q = data_queue.get()
            # In your visualization_thread, print the length of q to check it
            # print(f"Quaternion length: {len(q)}")  # This should be 4
            # Rotate cube vertices
            # rotated_vertices = rotate_cube(cube_vertices, roll, pitch, yaw)
            # rotated_vertices = rotate_cube_with_quaternion(cube_vertices, quaternion_multiply(quaternion_inverse(q),q0))
            rotated_vertices = rotate_cube_with_quaternion(cube_vertices, q)

            faces = get_cube_faces(rotated_vertices)

            # Clear the plot
            ax.clear()

            # Plot the cube
            ax.add_collection3d(Poly3DCollection(faces, facecolors=face_colors, linewidths=1, edgecolors='r', alpha=.25))

            # Set the axes limits
            ax.set_xlim([-cube_size, cube_size])
            ax.set_ylim([-cube_size, cube_size])
            ax.set_zlim([-cube_size, cube_size])
            
            #             # Assuming q is [w, x, y, z] quaternion format
            # pitch = np.arctan2(2.0 euler_angles = quaternion_to_euler(q)
            
            # euler_angles = quaternion_to_euler(quaternion_multiply(quaternion_inverse(q),q0))
            euler_angles = quaternion_to_euler(q)
            roll, pitch, yaw = euler_angles 
                    
            # # Convert to degrees
            yaw = np.degrees(yaw)
            pitch = np.degrees(pitch)
            roll = np.degrees(roll)   


            # q_test=quaternion_multiply(quaternion_inverse(q),q0)

                # Define your quaternion (w, x, y, z)
            # q_py = Quaternion(*q_test)  # Example quaternion

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            # euler_angles = q_py.yaw_pitch_roll  # This returns yaw, pitch, roll (in radians)

            # Extract the angles
            # yaw = euler_angles[0]  # Yaw angle (rotation around Z-axis)
            # pitch = euler_angles[1]  # Pitch angle (rotation around Y-axis)
            # roll = euler_angles[2]  # Roll angle (rotation around X-axis)

            # Optionally, convert radians to degrees
            yaw_deg = np.degrees(yaw)
            pitch_deg = np.degrees(pitch)
            roll_deg = np.degrees(roll)

            print(f"Yaw: {yaw_deg}, Pitch: {pitch_deg}, Roll: {roll_deg}")

            rate2 = rate2 + 1
            if rate2 == 2:
                print(f"Roll: {pitch}, Pitch: {roll}, Yaw: {yaw}")
                # print(f"Quaternion: {q}")
                # print(accel, gyro, mag)
                rate2 = 0

            # Set labels and title
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            # ax.set_title('Cube Rotation using Quaternion')

            ax.set_title(f'Cube Rotation - Roll: {pitch:.2f}, Pitch: {roll:.2f}, Yaw: {yaw:.2f}')

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
acc_misalignment, acc_scale, acc_bias = load_calibration('./../Arastronaut/6G 2/test_imu_acc.calib')
gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./../Arastronaut/6G 2/test_imu_gyro.calib')

# Example calibration offsets and scales for magnetometer
# mag_offsets = np.array([585.0, 310.0, 40.0])  # Offsets for X, Y, Z axes
# mag_offsets = np.array([86.0, 141.0, -105.0])  # Offsets for X, Y, Z axes

# mag_scales = np.array([3000.0, 3000.0, 3000.0])  # Scales for X, Y, Z axes
# mag_scales = np.array([1142.0, 1166.0, 1156.0])  # Scales for X, Y, Z axes
# mag_scales = np.array([2284.0, 2332.0, 2312.0])  # Scales for X, Y, Z axes

# Define calibration parameters
mag_offsets = np.array([308.7566, 441.8806, -649.8822])  # Hard iron correction bias
mag_scales = np.array([[1.0476, -0.0247, -0.0102],        # Soft iron correction matrix
                       [-0.0247, 0.9482, 0.0525],
                       [-0.0102, 0.0525, 1.0103]])
# Define scale factors (from user input)
scale_factors = np.array([5468.5, 6540, 5988.5])  # Example scaling factors for X, Y, Z axes

# Create a queue to pass data between threads
data_queue = queue.Queue()

# Start data collection thread
data_thread = threading.Thread(target=data_thread, args=(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales))
data_thread.daemon = True
data_thread.start()

# Start visualization in the main thread
visualization_thread(data_queue)
