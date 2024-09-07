import numpy as np
import socket
import struct
import queue
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import time 

# Initialize the Madgwick filter
madgwick = Madgwick()

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

# Example calibration offsets and scales for magnetometer
mag_offsets = np.array([537.0, 214.0, 55.0])  # Offsets for X, Y, Z axes
mag_scales = np.array([2510.0, 2172.0, 2356.0])  # Scales for X, Y, Z axes

# Load calibration data for accelerometer and gyroscope
acc_misalignment, acc_scale, acc_bias = load_calibration('./main server GUI/calib_data/test_imu_acc6G.calib')
gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./main server GUI/calib_data/test_imu_gyro.calib')

# Function to apply calibration to accelerometer and gyroscope data
def apply_calibration(accel, gyro):
    global acc_misalignment, acc_scale, acc_bias
    global gyro_misalignment, gyro_scale, gyro_bias

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

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on {UDP_IP}:{UDP_PORT}")

# Initialize the Madgwick MARG filter
madgwick = Madgwick()

# Initialize the quaternion (unit quaternion, representing no initial rotation)
q = np.array([1.0, 0.0, 0.0, 0.0])


# Main loop for receiving and processing data
rate = 0
rate2 = 0
prev_time = 0

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

                    # Apply calibration to accelerometer and gyroscope data
                    accel, gyro = apply_calibration(accel, gyro)

                    # Calibrate magnetometer data
                    mag = calibrate_data(mag, mag_offsets, mag_scales)

                    # Get the time step (dt) since the last reading
                    current_time = time.time()
                    dt = current_time - prev_time
                    prev_time = current_time

                    # Update Madgwick filter with new sensor data
                    # If you have magnetometer data, use `updateMARG`; otherwise use `updateIMU`
                    q = madgwick.updateMARG(q=q, gyr=gyro, acc=accel, mag=mag)

                    if q is not None:
                        # Output the quaternion
                        # Assuming quaternion q = [w, x, y, z]
                        w, x, y, z = q      

                        # Roll (rotation around X-axis)
                        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))

                        # Pitch (rotation around Y-axis)
                        sin_pitch = 2.0 * (w * y - z * x)
                        if np.abs(sin_pitch) >= 1:
                            pitch = np.sign(sin_pitch) * np.pi / 2  # Use 90 degrees if out of range
                        else:
                            pitch = np.arcsin(sin_pitch)

                        # Yaw (rotation around Z-axis)
                        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

                        # Convert to degrees if needed
                        yaw_deg = np.degrees(yaw)
                        pitch_deg = np.degrees(pitch)
                        roll_deg = np.degrees(roll)
                        rate2 = rate2 + 1
                        if rate2 == 10:
                            print(f"Roll: {roll_deg}, Pitch: {pitch_deg}, Yaw: {yaw_deg}")
                            print(f"Quaternion: {q}")
                            print(accel, gyro, mag)
                            rate2 = 0

    except socket.timeout:
        print("Waiting for data...")
    except KeyboardInterrupt:
        print("Terminating program...")
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
