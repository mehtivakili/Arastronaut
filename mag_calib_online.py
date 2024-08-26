import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import threading

rate = 0

# Configuration for the UDP receiver
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 12346  # Port to listen on (must match the ESP32 udpPort)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

# Global variables for data storage and offsets
magnetometer_data = []
calibrated_data = []
offsets = np.array([0.0, 0.0, 0.0])

def remove_outliers_and_calibrate(data):
    if len(data) == 0:
        return np.array([0.0, 0.0, 0.0]), data
    
    data = np.array(data)
    mean = np.mean(data, axis=0)
    std_dev = np.std(data, axis=0)
    
    z_scores = np.abs((data - mean) / std_dev)
    data_filtered = data[(z_scores < 2).all(axis=1)]
    
    offsets = np.mean(data_filtered, axis=0)
    data_corrected = data_filtered - offsets
    
    return offsets, data_corrected

def receive_udp_data():
    global rate
    print("UDP thread started.")
    check = "img/"
    check_encoded = check.encode()

    while True:
        data, addr = sock.recvfrom(4096)
        # print(f"Received packet from {addr}")

        parts = data.split(check_encoded)
        for part in parts:
            if len(part) == 44:
                rate += 1
                values = struct.unpack('<q9f', part)
                _, _, _, _, _, _, _, magX, magY, magZ = values
                if rate % 10 == 0:
                    magnetometer_data.append([magX, magY, magZ])

def update_plot(frame):
    global offsets, magnetometer_data, calibrated_data
    
    if len(magnetometer_data) == 0:
        return
    
    offsets, corrected_data = remove_outliers_and_calibrate(magnetometer_data)
    calibrated_data = corrected_data
    
    ax.clear()
    raw_data = np.array(magnetometer_data)
    if len(raw_data) > 0:
        ax.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], c='r', marker='o', label='Raw Data')
    if len(calibrated_data) > 0:
        ax.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2], c='g', marker='^', label='Calibrated Data')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('Real-Time Magnetometer Data')
    
    # Update the offset text with the latest offsets
    offset_text.set_text(f"Offsets - X: {offsets[0]:.2f}, Y: {offsets[1]:.2f}, Z: {offsets[2]:.2f}")
    print(f"Offsets - X: {offsets[0]:.2f}, Y: {offsets[1]:.2f}, Z: {offsets[2]:.2f}")

    ax.legend()

# Set up the plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
offset_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)

# Start the animation for real-time plotting
ani = FuncAnimation(fig, update_plot, interval=100)

# Start the UDP data reception in the background
udp_thread = threading.Thread(target=receive_udp_data, daemon=True)
udp_thread.start()

# Show the plot
plt.show()
