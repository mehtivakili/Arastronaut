import numpy as np
import socket
import struct
import threading
import queue
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from ahrs.filters import Madgwick
from ahrs.common import orientation
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

def initialize_orientation(accel, mag):
    """Initialize orientation using accelerometer and magnetometer without Euler angles."""
    # Use orientation.am2q to compute initial quaternion
    q0 = orientation.am2q(accel, mag)
    return q0

def quaternion_to_axis_angle(q):
    """Convert quaternion [w, x, y, z] to axis-angle representation."""
    qw, qx, qy, qz = q
    angle = 2 * np.arccos(qw)
    s = np.sqrt(1 - qw**2)
    if s < 1e-8:
        # If s is close to zero, axis can be any normalized vector
        x = 1.0
        y = 0.0
        z = 0.0
    else:
        x = qx / s
        y = qy / s
        z = qz / s
    axis = np.array([x, y, z])
    return axis, angle

def quaternion_conjugate(q):
    """Compute the conjugate of a quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def quaternion_multiply(q1, q2):
    """Multiply two quaternions."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return np.array([w, x, y, z])

# Visualization function
def visualization_thread(data_queue):
    # Initialize plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])
    # Define cube size and vertices
    cube_size = 1.0
    cube_vertices = create_cube(cube_size)
    face_colors = ['blue', 'red', 'yellow', 'blue', 'red', 'yellow']
    initial_quaternion = None  # To store the initial quaternion
    while True:
        try:
            # Get quaternion from the queue
            q = data_queue.get()
            if initial_quaternion is None:
                initial_quaternion = q
            # Compute relative rotation: q_relative = q_current * q_initial_conj
            q_initial_conj = quaternion_conjugate(initial_quaternion)
            q_relative = quaternion_multiply(q, q_initial_conj)
            # Convert relative quaternion to rotation object
            rotation = R.from_quat([q_relative[1], q_relative[2], q_relative[3], q_relative[0]])  # [x, y, z, w]
            # Rotate cube vertices
            rotated_vertices = rotation.apply(cube_vertices)
            faces = get_cube_faces(rotated_vertices)
            # Clear the plot
            ax.clear()
            # Plot the cube
            ax.add_collection3d(Poly3DCollection(faces, facecolors=face_colors, linewidths=1, edgecolors='r', alpha=.25))
            # Set the axes limits
            ax.set_xlim([-cube_size*2, cube_size*2])
            ax.set_ylim([-cube_size*2, cube_size*2])
            ax.set_zlim([-cube_size*2, cube_size*2])
            # Compute axis and angle from relative rotation
            axis, angle = quaternion_to_axis_angle(q_relative)
            angle_deg = np.degrees(angle)
            # Plot initial and current frames using quivers
            origin = np.array([0, 0, 0])
            # Initial frame axes
            ax.quiver(*origin, 1, 0, 0, color='r', length=1.0, normalize=True)
            ax.quiver(*origin, 0, 1, 0, color='g', length=1.0, normalize=True)
            ax.quiver(*origin, 0, 0, 1, color='b', length=1.0, normalize=True)
            # Current frame axes
            x_axis_rotated = rotation.apply([1, 0, 0])
            y_axis_rotated = rotation.apply([0, 1, 0])
            z_axis_rotated = rotation.apply([0, 0, 1])
            ax.quiver(*origin, *x_axis_rotated, color='r', length=1.0, normalize=True, linestyle='dashed')
            ax.quiver(*origin, *y_axis_rotated, color='g', length=1.0, normalize=True, linestyle='dashed')
            ax.quiver(*origin, *z_axis_rotated, color='b', length=1.0, normalize=True, linestyle='dashed')
            # Set labels and title
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(f'Rotation angle: {angle_deg:.2f}°, Axis: [{axis[0]:.2f}, {axis[1]:.2f}, {axis[2]:.2f}]')
            plt.pause(0.01)  # Pause to update the plot
        except queue.Empty:
            pass

# Data processing function
def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias,
                gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales, scale_factors):
    previous_timestamp_s = 0.0
    init_rot = True
    q = np.array([1.0, 0.0, 0.0, 0.0])  # Initialize quaternion
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
                    accel = np.array([accelX, accelY, accelZ])
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
                    if init_rot:
                        # Initialize the orientation using accelerometer and magnetometer
                        initial_quaternion = initialize_orientation(accel, mag)
                        q = initial_quaternion
                        # Initialize Madgwick filter with initial quaternion
                        madgwick = Madgwick(beta=0.8, frequency=200, q0=q)
                        init_rot = False
                    else:
                        # Update Madgwick filter with new sensor data and pass current quaternion
                        q = madgwick.updateMARG(q=q, gyr=gyro, acc=accel, mag=mag)
                    if q is not None:
                        # Send quaternion to the visualization thread
                        data_queue.put(q)
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
acc_misalignment, acc_scale, acc_bias = load_calibration('./../Arastronaut/main server GUI/calib_data/test_imu_acc6G.calib')
gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./../Arastronaut/main server GUI/calib_data/test_imu_gyro6G.calib')

## Fixed the axis issue and mag and imu are aligned
mag_offsets = np.array([1463.4, 2316.1, -382.8])  # Hard iron correction bias
mag_scales = np.array([[1.0207, 0.00315, 0.0538],        # Soft iron correction matrix
                       [-0.0315, 0.9730, 0.0980],
                       [0.0538, 0.0980, 1.0209]])
# Define scale factors (from user input)
scale_factors = np.array([6759.5, 6077, 5799.5])  # Example scaling factors for X, Y, Z axes

# Create a queue to pass data between threads
data_queue = queue.Queue()

# Start data collection thread
data_thread = threading.Thread(target=data_thread, args=(
    sock, data_queue, acc_misalignment, acc_scale, acc_bias,
    gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales, scale_factors))
data_thread.daemon = True
data_thread.start()

# Start visualization in the main thread
visualization_thread(data_queue)
