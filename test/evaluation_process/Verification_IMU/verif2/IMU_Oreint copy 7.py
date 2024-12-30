import numpy as np
import socket
import struct
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import time
import csv
from pyquaternion import Quaternion

# Constants
MAG_LOWER_BOUND = -100000
MAG_UPPER_BOUND = 100000

# Shared variables and locks
shared_data = {
    'quaternion': None,
    'accelerometer': None,
    'magnetometer': None
}
data_lock = threading.Lock()

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

def euler2quat(roll, pitch, yaw):
    """Convert roll, pitch, yaw (in radians) to quaternion [w, x, y, z]."""
    r = R.from_euler('xyz', [roll, pitch, yaw])
    q = r.as_quat()  # Returns [x, y, z, w]
    q = np.array([q[3], q[0], q[1], q[2]])  # Rearrange to [w, x, y, z]
    return q

def initialize_orientation(accel, mag):
    """Initialize orientation using accelerometer and magnetometer."""
    # Normalize accelerometer and magnetometer readings
    accel_norm = accel / np.linalg.norm(accel)
    mag_norm = mag / np.linalg.norm(mag)
    # Calculate pitch and roll from accelerometer
    pitch = np.arcsin(-accel_norm[0])
    roll = np.arctan2(accel_norm[1], accel_norm[2])
    # Adjust magnetometer by pitch and roll to get correct yaw
    mag_x = (mag_norm[0] * np.cos(pitch) +
             mag_norm[2] * np.sin(pitch))
    mag_y = (mag_norm[0] * np.sin(roll) * np.sin(pitch) +
             mag_norm[1] * np.cos(roll) -
             mag_norm[2] * np.sin(roll) * np.cos(pitch))
    yaw = np.arctan2(-mag_y, mag_x)
    # Convert to quaternion
    initial_quaternion = euler2quat(roll, pitch, yaw)
    return initial_quaternion

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

def rotate_vector(q, v):
    """
    Rotate vector v by quaternion q.
    q: [w, x, y, z]
    v: [x, y, z]
    Returns rotated vector.
    """
    q_conj = quaternion_conjugate(q)
    v_quat = np.array([0] + list(v))
    rotated_v = quaternion_multiply(quaternion_multiply(q, v_quat), q_conj)
    return rotated_v[1:]  # Return the vector part

# Visualization function
def visualization_thread():
    # Initialize plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])
    plt.ion()
    fig.show()

    # Define cube size and vertices
    cube_size = 1.0
    cube_vertices = create_cube(cube_size)
    faces = get_cube_faces(cube_vertices)
    face_colors = ['blue', 'red', 'yellow', 'blue', 'red', 'yellow']

    # Create Poly3DCollection and add to axes
    cube_poly = Poly3DCollection(faces, facecolors=face_colors, linewidths=1, edgecolors='r', alpha=0.25)
    ax.add_collection3d(cube_poly)

    # Set the axes limits
    ax.set_xlim([-cube_size*2, cube_size*2])
    ax.set_ylim([-cube_size*2, cube_size*2])
    ax.set_zlim([-cube_size*2, cube_size*2])

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Create lines for initial frame axes (world frame)
    line_x_init, = ax.plot([0,1], [0,0], [0,0], color='r', label='East')
    line_y_init, = ax.plot([0,0], [0,1], [0,0], color='g', label='North')
    line_z_init, = ax.plot([0,0], [0,0], [0,1], color='b', label='Up')

    # Create lines for current frame axes (sensor frame)
    line_x_curr, = ax.plot([0,1], [0,0], [0,0], color='r', linestyle='dashed', label='Sensor X')
    line_y_curr, = ax.plot([0,0], [0,1], [0,0], color='g', linestyle='dashed', label='Sensor Y')
    line_z_curr, = ax.plot([0,0], [0,0], [0,1], color='b', linestyle='dashed', label='Sensor Z')

    # Add legend
    ax.legend(loc='upper left')

    # Initialize variables
    initial_quaternion = None

    # Quiver handles for accelerometer and magnetometer vectors
    accel_quiver = ax.quiver(0, 0, 0, 0, 0, 0, color='k', length=1.0, normalize=True, label='Accelerometer')
    mag_quiver = ax.quiver(0, 0, 0, 0, 0, 0, color='m', length=1.0, normalize=True, label='Magnetometer')
    


    # To manage labels for quivers in the legend, we'll create proxy artists
    accel_proxy = plt.Line2D([0], [0], linestyle="none", marker='^', markersize=10, markerfacecolor='k', label='Accelerometer')
    mag_proxy = plt.Line2D([0], [0], linestyle="none", marker='^', markersize=10, markerfacecolor='m', label='Magnetometer')
    ax.legend([line_x_init, line_y_init, line_z_init, accel_proxy, mag_proxy],
              ['East', 'North', 'Up', 'Accelerometer', 'Magnetometer'],
              loc='upper left')

    while True:
        with data_lock:
            q = shared_data['quaternion']
            accel = shared_data['accelerometer']
            mag = shared_data['magnetometer']

        if q is None or accel is None or mag is None:
            # No data yet
            time.sleep(0.01)
            continue

        if initial_quaternion is None:
            initial_quaternion = q
            q_initial_conj = quaternion_conjugate(initial_quaternion)

        # Compute relative rotation
        q_relative = quaternion_multiply(q, q_initial_conj)
        q_relative = normalize_quaternion(q_relative)

        # Rotate cube vertices
        rotated_vertices = rotate_cube_with_quaternion(cube_vertices, q_relative)
        faces = get_cube_faces(rotated_vertices)

        # Update the Poly3DCollection's vertices
        cube_poly.set_verts(faces)

        # Compute rotated axes
        x_axis_rotated = rotate_vector(q_relative, [1, 0, 0])
        y_axis_rotated = rotate_vector(q_relative, [0, 1, 0])
        z_axis_rotated = rotate_vector(q_relative, [0, 0, 1])

        # Update current frame axes lines
        line_x_curr.set_data([0, x_axis_rotated[0]], [0, x_axis_rotated[1]])
        line_x_curr.set_3d_properties([0, x_axis_rotated[2]])

        line_y_curr.set_data([0, y_axis_rotated[0]], [0, y_axis_rotated[1]])
        line_y_curr.set_3d_properties([0, y_axis_rotated[2]])

        line_z_curr.set_data([0, z_axis_rotated[0]], [0, z_axis_rotated[1]])
        line_z_curr.set_3d_properties([0, z_axis_rotated[2]])

        # Update accelerometer vector
        accel_norm = accel / np.linalg.norm(accel)
        accel_quiver.remove()
        accel_quiver = ax.quiver(0, 0, 0, accel_norm[0], accel_norm[1], accel_norm[2],
                                 color='k', length=1.0, normalize=True, label='Accelerometer')

        # Update magnetometer vector
        mag_norm = mag / np.linalg.norm(mag)
        mag_quiver.remove()
        mag_quiver = ax.quiver(0, 0, 0, mag_norm[0], mag_norm[1], mag_norm[2],
                               color='m', length=1.0, normalize=True, label='Magnetometer')
        # Compute angle and axis for CSV writing
        q_relative = q_relative/np.linalg.norm(q_relative)
        axis, angle = Quaternion(q_relative).axis, Quaternion(q_relative).degrees
        
        # Update the plot title with angle and axis
        ax.set_title(f'Rotation angle: {angle:.2f}°, Axis: [{axis[0]:.2f}, {axis[1]:.2f}, {axis[2]:.2f}]')


        # Refresh the plot
        plt.draw()
        plt.pause(0.001)

# Data processing function
def data_thread(sock, acc_misalignment, acc_scale, acc_bias,
                gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales, scale_factors):
    previous_timestamp_s = 0.0
    init_rot = True
    count = 0
    madgwick = Madgwick(beta=0.8, frequency=200)

    # Open CSV file to save data
    with open('imu_mag_yaw.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(['timestamp_ns', 'angle_axis'])

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
                        accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]
                        # Convert sensor data into numpy arrays
                        accel = np.array([-accelY, -accelX, accelZ])
                        gyro = np.array([gyroY, gyroX, -gyroZ])
                        mag = np.array([magX, magY, magZ])
                        # Filter the magnetometer data
                        if not filter_magnetometer(mag, MAG_LOWER_BOUND, MAG_UPPER_BOUND):
                            continue
                        # Calibrate magnetometer data
                        mag_calib = calibrate_magnetometer(mag, mag_offsets, mag_scales, scale_factors=scale_factors)

                        if init_rot:
                            # Initialize the orientation using accelerometer and magnetometer
                            initial_quaternion = initialize_orientation(accel, mag_calib)
                            q = initial_quaternion
                            # Initialize Madgwick filter with initial quaternion
                            madgwick = Madgwick(q0=initial_quaternion, beta=0.8, frequency=200)
                            init_rot = False
                        else:
                            # Update Madgwick filter with new sensor data
                            q = madgwick.updateMARG(q=q ,gyr=gyro, acc=accel, mag=mag_calib)
                        if q is not None:
                            
                            system_timestamp_ns = int(time.time_ns())
                            # Save to CSV
                            # Compute angle and axis for CSV writing
                            axis, angle = Quaternion(q).axis, Quaternion(q).degrees
                            csvwriter.writerow([system_timestamp_ns, angle])
                            csvfile.flush()
                            print(accel)
                            # Update shared data under lock
                            with data_lock:
                                shared_data['quaternion'] = q
                                shared_data['accelerometer'] = accel
                                shared_data['magnetometer'] = mag_calib

            except socket.timeout:
                print("Waiting for data...")
            except KeyboardInterrupt:
                print("Terminating data thread...")
                break

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)
sock.bind((UDP_IP, UDP_PORT))

# Load calibration data for accelerometer and gyroscope
acc_misalignment, acc_scale, acc_bias = load_calibration('./../Arastronaut/calib_data/mpu_params/test_imu_acc.calib')
gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./../Arastronaut/calib_data/mpu_params/test_imu_gyro.calib')

# Define magnetometer calibration parameters
mag_offsets = np.array([ -3.0701, 62.3422, -31.4786])  # Hard iron correction bias
mag_scales = np.array([[ 0.9408,  0,    0],
                       [ 0,   1.0458,   0],
                       [ 0,    0,   1.0164]])
scale_factors = np.array([53.6609, 50.8387, 51.1362])  # Scaling factors for X, Y, Z axes

# Start data collection thread
data_thread_instance = threading.Thread(target=data_thread, args=(
    sock, acc_misalignment, acc_scale, acc_bias,
    gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales, scale_factors))
data_thread_instance.daemon = True
data_thread_instance.start()

# Start visualization thread
visualization_thread()
