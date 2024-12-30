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
import time
import csv
from pyquaternion import Quaternion
from ahrs.common.orientation import q2euler

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

def euler2quat(roll, pitch, yaw):
    """Convert roll, pitch, yaw (in radians) to quaternion [w, x, y, z]."""
    r = R.from_euler('xyz', [roll, pitch, yaw])
    q = r.as_quat()  # Returns [x, y, z, w]
    q = np.array([q[3], q[0], q[1], q[2]])  # Rearrange to [w, x, y, z]
    return q

def initialize_orientation(accel, mag):
    """Initialize orientation using accelerometer and magnetometer."""
    # acc0, acc1, acc2 = accel[0],accel[1],accel[2]
    # accel[1] = acc0
    # accel[0]=acc1
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

def quaternion_to_axis_angle(q):
    """Convert quaternion [w, x, y, z] to axis-angle representation."""
    qw, qx, qy, qz = normalize_quaternion(q)
    q = normalize_quaternion(q)
    r = Quaternion(q)
    angle = r.angle
    axis = r.axis
    # print(axis)
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
    initial_quaternion = None  # To store the initial quaternion

    # Define World Frame Vectors
    gravity_world = np.array([0, 0, -1])  # Gravity vector in world frame
    north_world = np.array([1, 0, 0])     # North vector in world frame (assuming X-East, Y-North)
    w = 0
    # Open CSV file to save data
    with open('dataset-screw3.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write the header
        csvwriter.writerow(['timestamp_ns', 'angle_axis'])
        while True:
            try:
                # Get quaternion from the queue
                q = data_queue.get(timeout=0.1)
                if initial_quaternion is None:
                    initial_quaternion = q
                    # print(f"q_init:{initial_quaternion}")
                    q_initial_conj = quaternion_conjugate(initial_quaternion)

                # Compute relative rotation: q_relative = q_current * q_initial_conj
                q_relative = quaternion_multiply(q, q_initial_conj)

                # Normalize the relative quaternion to prevent drift
                # q_relative = normalize_quaternion(q)
                q_relative = normalize_quaternion(q_relative)
                # Convert relative quaternion to rotation object
                # rotation = R.from_quat([q[1], q[2], q[3], q[0]])  # [x, y, z, w]
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
                print(angle_deg)
                w = w + 1
                system_timestamp_ns = int(time.time_ns())
                if (w == 10):
                    w = 0
                    # Save to CSV (only axis and angle)
                    csvwriter.writerow([system_timestamp_ns, angle_deg])
                    csvfile.flush()  # Ensure data is written to the file immediately

                # Plot initial and current frames using quivers
                origin = np.array([0, 0, 0])

                # Initial frame axes (world frame)
                ax.quiver(*origin, 1, 0, 0, color='r', length=1.0, normalize=True, label='East')
                ax.quiver(*origin, 0, 1, 0, color='g', length=1.0, normalize=True, label='North')
                ax.quiver(*origin, 0, 0, 1, color='b', length=1.0, normalize=True, label='Up')

                # Current frame axes (sensor frame)
                x_axis_rotated = rotation.apply([1, 0, 0])
                y_axis_rotated = rotation.apply([0, 1, 0])
                z_axis_rotated = rotation.apply([0, 0, 1])
                ax.quiver(*origin, *x_axis_rotated, color='r', length=1.0, normalize=True, linestyle='dashed', label='Sensor X')
                ax.quiver(*origin, *y_axis_rotated, color='g', length=1.0, normalize=True, linestyle='dashed', label='Sensor Y')
                ax.quiver(*origin, *z_axis_rotated, color='b', length=1.0, normalize=True, linestyle='dashed', label='Sensor Z')

                # Calculate and plot Gravity Vector
                gravity_sensor = rotate_vector([1,0,0,0], gravity_world)  # Rotate gravity vector to sensor frame
                ax.quiver(*origin, *gravity_sensor, color='k', length=1.0, normalize=True, label='Gravity')  # ADDED

                # Calculate and plot North Vector
                north_sensor = rotate_vector(q_relative, north_world)  # Rotate north vector to sensor frame
                ax.quiver(*origin, *north_sensor, color='m', length=1.0, normalize=True, label='North')  # ADDED

                # Set labels and title
                ax.set_xlabel('X')
                ax.set_ylabel('Y')
                ax.set_zlabel('Z')
                ax.set_title(f'Rotation angle: {angle_deg:.2f}°, Axis: [{axis[0]:.2f}, {axis[1]:.2f}, {axis[2]:.2f}]')

                # Add legend
                ax.legend(loc='upper left')

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
    init_rot = True
    count = 0
    madgwick = Madgwick(beta=0.5, frequency=200)

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
                    # print(f"Received sensor data: accel={accel}, gyro={gyro}, mag={mag}")  # Debug: Print sensor data
                    # print(f"Received sensor data: accel={accel}")  # Debug: Print sensor data
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
                        madgwick = Madgwick(beta=0.5, frequency=200)
                        init_rot = False
                    else:
                        # Update Madgwick filter with new sensor data and pass current quaternion
                        q = madgwick.updateMARG(q=q, gyr=gyro, acc=accel, mag=mag)
                        # print(f"Updated quaternion: {q}")  # Debug: Print updated quaternion
                    if q is not None:
                        # Get system timestamp in nanoseconds
                        euler = q2euler(q)
                        roll = euler[0]  # Roll angle
                        pitch = euler[1]  # Pitch angle
                        yaw = euler[2]  # Yaw angle
                        # Convert heading to degrees
                        heading_degrees_fuse = np.degrees(yaw)
                        if heading_degrees_fuse < 0:
                            heading_degrees_fuse += 360

                        count = count + 1
                        if count == 20:
                            # print(accel, gyro)
                            # print(f"Euler angles: Roll: {np.degrees(roll):.2f} degrees, Pitch: {np.degrees(pitch):.2f} degrees, Yaw: {np.degrees(yaw):.2f} radians")
                            count = 0

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
