import numpy as np
import socket
import struct
import threading
import queue
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from vqf import VQF
from scipy.spatial.transform import Rotation as R
import time
import csv
from matplotlib.animation import FuncAnimation

# Constants
MAG_LOWER_BOUND = -100000
MAG_UPPER_BOUND = 100000

# Helper Functions
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
        [ size, -size, -size],
        [ size,  size, -size],
        [-size,  size, -size],
        [-size, -size,  size],
        [ size, -size,  size],
        [ size,  size,  size],
        [-size,  size,  size]
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

def quaternion_to_axis_angle(q):
    """Convert quaternion [w, x, y, z] to axis-angle representation."""
    q = normalize_quaternion(q)
    r = R.from_quat([q[1], q[2], q[3], q[0]])  # [x, y, z, w]
    angle = r.magnitude()
    if angle == 0:
        axis = np.array([1, 0, 0])  # Default axis
    else:
        axis = r.as_rotvec() / angle
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

# Data Processing Function
def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias,
                gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales, scale_factors, stop_event):
    """Thread function for processing incoming IMU data."""
    # Initialize VQF filter
    sampling_time = 0.005  # 200 Hz
    vqf_filter = VQF(gyrTs=sampling_time, accTs=sampling_time, magTs=sampling_time)
    
    while not stop_event.is_set():
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
                    # timestamp_s = timestamp_ns / 1e9  # Convert nanoseconds to seconds if needed
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

                    if q is not None:
                        # Put the quaternion into the queue for visualization
                        try:
                            data_queue.put_nowait(q)
                        except queue.Full:
                            pass  # Drop the data if the queue is full to maintain real-time performance

        except socket.timeout:
            continue  # Continue waiting for data
        except Exception as e:
            print(f"Data Thread Error: {e}")
            break

# Visualization Function
def update_plot(frame, data_queue, poly3d, quivers_sensor, quivers_gravity_north, initial_quaternion):
    """Update function for FuncAnimation."""
    try:
        q = data_queue.get_nowait()
    except queue.Empty:
        return poly3d, quivers_sensor, quivers_gravity_north

    if initial_quaternion[0] is None:
        initial_quaternion[0] = q
        q_initial_conj = quaternion_conjugate(initial_quaternion[0])
        return poly3d, quivers_sensor, quivers_gravity_north

    # Compute relative rotation: q_relative = q_current * q_initial_conj
    q_initial_conj = quaternion_conjugate(initial_quaternion[0])
    q_relative = quaternion_multiply(q, q_initial_conj)
    q_relative = normalize_quaternion(q_relative)

    # Convert relative quaternion to rotation object
    rotation = R.from_quat([q_relative[1], q_relative[2], q_relative[3], q_relative[0]])  # [x, y, z, w]

    # Rotate cube vertices
    rotated_vertices = rotation.apply(cube_vertices)
    rotated_faces = get_cube_faces(rotated_vertices)

    # Update cube faces
    poly3d.set_verts(rotated_faces)

    # Update sensor frame axes
    sensor_axes_rotated = rotation.apply(np.identity(3))
    for i in range(3):
        quivers_sensor[i].remove()
        quivers_sensor[i] = ax.quiver(*origin, *sensor_axes_rotated[i], color=sensor_colors[i],
                                     length=1.0, normalize=True, linestyle='dashed')

    # Rotate and plot gravity vector
    gravity_sensor = rotate_vector(q_relative, gravity_world)
    quivers_gravity_north['gravity'].remove()
    quivers_gravity_north['gravity'] = ax.quiver(*origin, *gravity_sensor, color='k', length=1.0, normalize=True, label='Gravity')

    # Rotate and plot north vector
    north_sensor = rotate_vector(q_relative, north_world)
    quivers_gravity_north['north'].remove()
    quivers_gravity_north['north'] = ax.quiver(*origin, *north_sensor, color='m', length=1.0, normalize=True, label='North')

    # Compute axis and angle from relative rotation
    axis, angle = quaternion_to_axis_angle(q_relative)
    angle_deg = np.degrees(angle)

    # Update plot title
    ax.set_title(f'Rotation angle: {angle_deg:.2f}°, Axis: [{axis[0]:.2f}, {axis[1]:.2f}, {axis[2]:.2f}]')

    # Log to CSV every 10 frames
    update_plot.frame_count += 1
    if update_plot.frame_count >= 1:
        system_timestamp_ns = int(time.time_ns())
        csv_writer.writerow([system_timestamp_ns, angle_deg])
        csv_file.flush()
        update_plot.frame_count = 0

    return poly3d, quivers_sensor, quivers_gravity_north

update_plot.frame_count = 0  # Initialize frame count

# Main Execution
if __name__ == "__main__":
    # UDP Setup
    UDP_IP = "0.0.0.0"
    UDP_PORT = 12346
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))
    sock.settimeout(1.0)  # Set timeout to allow graceful shutdown

    # Load Calibration Data for Accelerometer and Gyroscope
    acc_calib_file = './../Arastronaut/calib_data/mpu_params2/test_imu_acc.calib'
    gyro_calib_file = './../Arastronaut/calib_data/mpu_params2/test_imu_gyro.calib'
    acc_misalignment, acc_scale, acc_bias = load_calibration(acc_calib_file)
    gyro_misalignment, gyro_scale, gyro_bias = load_calibration(gyro_calib_file)

    # Define Magnetometer Calibration Parameters
    mag_offsets = np.array([-2.1702, 61.6808, -32.3492])  # Hard iron correction bias
    mag_scales = np.array([
        [0.9494, -0.0026, 0.0446],
        [-0.0026, 1.0267, 0.0145],
        [0.0446, 0.0145, 1.0380]
    ])
    scale_factors = np.array([70.851, 69.54, 62.782])  # Example scaling factors for X, Y, Z axes

    # Create a queue to pass data between threads
    data_queue = queue.Queue(maxsize=2)  # Adjust maxsize as needed

    # Event to signal threads to stop
    stop_event = threading.Event()

    # Start Data Collection Thread
    data_thread_instance = threading.Thread(target=data_thread, args=(
        sock, data_queue, acc_misalignment, acc_scale, acc_bias,
        gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales, scale_factors, stop_event))
    data_thread_instance.daemon = True
    data_thread_instance.start()

    # Initialize Plot
    plt.style.use('ggplot')
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_box_aspect([1, 1, 1])

    # Define cube size and vertices
    cube_size = 1.0
    cube_vertices = create_cube(cube_size)
    faces = get_cube_faces(cube_vertices)
    face_colors = ['cyan', 'magenta', 'yellow', 'cyan', 'magenta', 'yellow']

    # Create Poly3DCollection for the cube
    poly3d = Poly3DCollection(faces, facecolors=face_colors, linewidths=1, edgecolors='k', alpha=0.25)
    ax.add_collection3d(poly3d)

    # Set the axes limits
    ax.set_xlim([-cube_size*2, cube_size*2])
    ax.set_ylim([-cube_size*2, cube_size*2])
    ax.set_zlim([-cube_size*2, cube_size*2])

    # Initialize quivers for world frame
    origin = np.array([0, 0, 0])
    world_axes = np.identity(3)
    world_colors = ['r', 'g', 'b']
    quivers_world = []
    for i in range(3):
        quivers_world.append(ax.quiver(*origin, *world_axes[i], color=world_colors[i],
                                      length=1.0, normalize=True, label=f'World {["X","Y","Z"][i]}'))

    # Initialize quivers for sensor frame
    sensor_colors = ['r', 'g', 'b']
    quivers_sensor = []
    for i in range(3):
        quivers_sensor.append(ax.quiver(*origin, *world_axes[i], color=sensor_colors[i],
                                       length=1.0, normalize=True, linestyle='dashed', label=f'Sensor {["X","Y","Z"][i]}'))

    # Initialize vectors for gravity and north
    gravity_world = np.array([0, 0, -1])  # Gravity vector in world frame
    north_world = np.array([1, 0, 0])     # North vector in world frame (assuming X-East, Y-North)
    quivers_gravity_north = {}
    quivers_gravity_north['gravity'] = ax.quiver(*origin, *gravity_world, color='k', length=1.0, normalize=True, label='Gravity')
    quivers_gravity_north['north'] = ax.quiver(*origin, *north_world, color='m', length=1.0, normalize=True, label='North')

    # Initialize CSV logging
    csv_file = open('dataset-screw30.csv', 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(['timestamp_ns', 'angle_deg'])

    # Initialize initial quaternion as a mutable object to allow updates within update_plot
    initial_quaternion = [None]

    # Define update_plot with additional arguments
    def wrapped_update_plot(frame):
        return update_plot(frame, data_queue, poly3d, quivers_sensor, quivers_gravity_north, initial_quaternion)

    # Create FuncAnimation
    ani = FuncAnimation(fig, wrapped_update_plot, interval=10, blit=False)

    # Start the animation
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        stop_event.set()
        data_thread_instance.join()
        csv_file.close()
        sock.close()
        print("Shutdown complete.")
