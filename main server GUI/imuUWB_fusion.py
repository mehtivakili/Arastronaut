import socket
import struct
import numpy as np

# Define UDP settings
UDP_IP = "192.168.4.100"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# Define anchors' positions (example coordinates)
anchors = np.array([[0, 0], [10, 0], [5, 10]])  # 3 anchors in a 2D plane

# Initialize the Kalman filter state vector (px, py, vx, vy)
x = np.zeros((4, 1))  # Initial state [px, py, vx, vy]
P = np.eye(4)  # State covariance matrix
Q = np.eye(4) * 0.01  # Process noise covariance matrix
R = np.eye(3) * 0.1  # Measurement noise covariance matrix for UWB

# State transition matrix
dt = 1 / 200  # IMU update rate
F = np.eye(4)
F[0, 2] = dt
F[1, 3] = dt

def h(x, anchors):
    distances = np.sqrt((x[0, 0] - anchors[:, 0])**2 + (x[1, 0] - anchors[:, 1])**2)
    return distances.reshape(-1, 1)

def calculate_jacobian(x, anchors):
    H = np.zeros((3, 4))
    for i in range(3):
        diff_x = x[0, 0] - anchors[i, 0]
        diff_y = x[1, 0] - anchors[i, 1]
        distance = np.sqrt(diff_x**2 + diff_y**2)
        H[i, 0] = diff_x / distance
        H[i, 1] = diff_y / distance
    return H

def predict_with_imu(x, F, dt, imu_data):
    ax, ay = imu_data
    x[2, 0] += ax * dt  # Update vx
    x[3, 0] += ay * dt  # Update vy
    x_pred = np.dot(F, x)
    return x_pred

def kalman_filter_step(x, P, z_uwb, F, Q, R, anchors, imu_data):
    x_pred = predict_with_imu(x, F, dt, imu_data)
    P_pred = np.dot(F, np.dot(P, F.T)) + Q

    z_pred = h(x_pred, anchors)
    y = z_uwb - z_pred
    H = calculate_jacobian(x_pred, anchors)
    S = np.dot(H, np.dot(P_pred, H.T)) + R
    K = np.dot(P_pred, np.dot(H.T, np.linalg.inv(S)))
    x_update = x_pred + np.dot(K, y)
    P_update = P_pred - np.dot(K, np.dot(H, P_pred))

    return x_update, P_update

def parse_imu_data(data):
    timestamp = struct.unpack('f', data[0:4])[0]
    accelX = struct.unpack('f', data[4:8])[0]
    accelY = struct.unpack('f', data[8:12])[0]
    accelZ = struct.unpack('f', data[12:16])[0]
    gyroX = struct.unpack('f', data[16:20])[0]
    gyroY = struct.unpack('f', data[20:24])[0]
    gyroZ = struct.unpack('f', data[24:28])[0]
    return timestamp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ

def parse_uwb_data(data):
    time = struct.unpack('f', data[0:4])[0]
    address = struct.unpack('f', data[4:8])[0]
    distance = struct.unpack('f', data[8:12])[0]
    return time, address, distance

# Initialize a dictionary to store distances to each anchor
uwb_distances = {130: None, 131: None, 133: None}

# Log every step
import logging
logging.basicConfig(level=logging.DEBUG)

while True:
    data, addr = sock.recvfrom(1024)
    if len(data) > 0:
        if data.startswith(b'abc/'):
            _, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = parse_imu_data(data[4:])
            imu_data = [accelX, accelY]
            # logging.debug(f"IMU Data: Accel=({accelX},{accelY},{accelZ}), Gyro=({gyroX},{gyroY},{gyroZ})")
            # Predict step using IMU data
            x = predict_with_imu(x, F, dt, imu_data)
            # logging.debug(f"Predicted State: {x.flatten()}")
        
        elif data.startswith(b'cba/'):
            _, address, distance = parse_uwb_data(data[4:])
            uwb_distances[int(address)] = distance
            logging.debug(f"UWB Data: Address = {address}, Distance={distance}")
            
            if None not in uwb_distances.values():
                # Perform the Kalman filter update step once we have distances from all anchors
                z_uwb = np.array([[uwb_distances[130]], [uwb_distances[131]], [uwb_distances[133]]])
                # logging.debug(f"UWB Measurement Vector: {z_uwb.flatten()}")
                x, P = kalman_filter_step(x, P, z_uwb, F, Q, R, anchors, imu_data)
                # logging.debug(f"Updated State: {x.flatten()}")

                # print(f"Updated Position: ({x[0, 0]}, {x[1, 0]})")

                # Reset UWB distances after the update
                uwb_distances = {130: None, 131: None, 133: None}
