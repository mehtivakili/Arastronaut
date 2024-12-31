import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import socket
import struct
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import threading

# Complementary Filter Parameters
alpha = 0.98  # Higher alpha means more trust in the gyroscope data
acc_weight = 1.0  # Weight for the accelerometer

# Initial quaternion (identity, meaning no rotation)
q = np.array([1, 0, 0, 0])

# Calibration matrices and offsets (will be loaded later)
acc_misalignment = None
acc_scale = None
acc_bias = None
gyro_misalignment = None
gyro_scale = None
gyro_bias = None

# Global variable for quaternion data storage
latest_quaternion = np.array([1, 0, 0, 0])  # Initial quaternion

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

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm

def complementary_filter(q, gyro, acc, dt):
    global alpha, acc_weight

    # Integrate the gyroscope data to get the new orientation
    gyro = gyro * np.pi / 180.0  # Convert to radians
    delta_rotation = R.from_rotvec(gyro * dt)
    q_gyro = delta_rotation * R.from_quat(q)
    
    # Normalize accelerometer data and use it to estimate gravity direction
    acc = normalize(acc)
    
    # Estimate the orientation based on the accelerometer
    acc_quat = np.array([0, acc[0], acc[1], acc[2]])
    gravity_direction = R.from_quat(acc_quat).inv()
    q_acc = gravity_direction * R.from_quat(np.array([0, 0, 0, 1]))  # Align Z-axis to gravity
    
    # Blend the gyro and accelerometer results manually
    q_blended = q_gyro.as_quat() * alpha + q_acc.as_quat() * (1 - alpha)

    # Normalize the resulting quaternion
    return normalize(q_blended)

def receive_udp_data():
    global latest_quaternion
    UDP_IP = "0.0.0.0"
    UDP_PORT = 12346

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP packets on port {UDP_PORT}...")

    prev_time = None

    while True:
        data, addr = sock.recvfrom(4096)

        check = "abc/"
        check_encoded = check.encode()
        parts = data.split(check_encoded)

        for part in parts:
            if len(part) == 32:
                values = struct.unpack('<q6f', part)
                Tio, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = values

                accel = np.array([accelX, accelY, accelZ])
                gyro = np.array([gyroX, gyroY, gyroZ])

                if (acc_misalignment is not None and acc_scale is not None and acc_bias is not None) and \
                   (gyro_misalignment is not None and gyro_scale is not None and gyro_bias is not None):
                    accel, gyro = apply_calibration(accel, gyro)

                timestamp = Tio / 1e9  # Convert Tio to seconds
                if prev_time is None:
                    prev_time = timestamp

                dt = timestamp - prev_time
                prev_time = timestamp

                # Apply the complementary filter
                global q
                q = complementary_filter(q, gyro, accel, dt)

                # Update the latest quaternion
                latest_quaternion = q.copy()

def update_plot(frame):
    global latest_quaternion

    ax.clear()

    # Convert quaternion to rotation matrix
    r = R.from_quat(latest_quaternion)
    rot_matrix = r.as_matrix()
    
    # Extract new axes
    x_axis = rot_matrix[:, 0]
    y_axis = rot_matrix[:, 1]
    z_axis = rot_matrix[:, 2]
    
    # Plot the axes representing the orientation
    ax.quiver(0, 0, 0, x_axis[0], x_axis[1], x_axis[2], color='r', label='X-axis')
    ax.quiver(0, 0, 0, y_axis[0], y_axis[1], y_axis[2], color='g', label='Y-axis')
    ax.quiver(0, 0, 0, z_axis[0], z_axis[1], z_axis[2], color='b', label='Z-axis')
    
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('Real-Time Orientation')

    ax.legend()

# Set up the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Start the animation for real-time plotting
ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

# Start the UDP data reception in the background
udp_thread = threading.Thread(target=receive_udp_data, daemon=True)
udp_thread.start()

# Show the plot
plt.show()

if __name__ == "__main__":
    acc_calibration_file = "./calib_data/test_imu_acc.calib"
    gyro_calibration_file = "./calib_data/test_imu_gyro.calib"

    acc_misalignment, acc_scale, acc_bias = load_calibration(acc_calibration_file)
    gyro_misalignment, gyro_scale, gyro_bias = load_calibration(gyro_calibration_file)

    # Start the UDP data reception and real-time plot
    plt.show()
