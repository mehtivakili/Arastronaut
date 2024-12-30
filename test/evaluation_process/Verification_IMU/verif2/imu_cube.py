import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial.transform import Rotation as R
import socket
import struct
import time

# Function to calculate roll, pitch, and yaw
def calculate_roll_pitch_yaw(accel, gyro, dt, prev_yaw):
    # Roll (rotation around X-axis)
    roll = np.arctan2(accel[1], accel[2])

    # Pitch (rotation around Y-axis)
    pitch = np.arctan2(-accel[0], np.sqrt(accel[1] ** 2 + accel[2] ** 2))

    # Yaw (rotation around Z-axis) integrated from gyroscope
    yaw = prev_yaw + gyro[2] * dt  # Integrate the z-axis gyro reading over time

    # Convert radians to degrees
    roll = np.degrees(roll)
    pitch = np.degrees(pitch)
    yaw = np.degrees(yaw)

    return roll, pitch, yaw

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

# Initialize plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1, 1, 1])

# Define cube size
cube_size = 1.0
cube_vertices = create_cube(cube_size)

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# Start time
start_time = time.time()
prev_yaw = 0.0  # Initialize previous yaw

# Main loop to update the cube
while True:
    data, addr = sock.recvfrom(4096)
    check = "abc/"
    check_encoded = check.encode()
    parts = data.split(check_encoded)

    for part in parts:
        if len(part) == 32:
            values = struct.unpack('<q6f', part)
            _, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = values

            accel = np.array([accelX, accelY, accelZ])
            gyro = np.array([gyroX, gyroY, gyroZ])

            # Calculate time difference
            current_time = time.time()
            dt = current_time - start_time
            start_time = current_time

            # Calculate roll, pitch, and yaw
            roll, pitch, yaw = calculate_roll_pitch_yaw(accel, gyro, dt, prev_yaw)
            prev_yaw = yaw  # Update the previous yaw

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

plt.show()
