import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import threading

rate = 0
buffer_size = 50  # Buffer size to hold recent points
diversity_threshold = 5.0  # Minimum distance to consider a point as diverse
motion_threshold = 0.5  # Threshold to detect significant motion
confidence_threshold = 90  # Confidence level required to finalize calibration

# Configuration for the UDP receiver
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 12346  # Port to listen on (must match the ESP32 udpPort)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow the socket to reuse the address
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

# Global variables for data storage, offsets, and buffer
magnetometer_data = []
calibrated_data = []
offsets = np.array([0.0, 0.0, 0.0])
buffer = []
last_point = np.array([0.0, 0.0, 0.0])
confidence = 0

def is_diverse(new_point, buffer, threshold):
    if not buffer:
        return True  # Buffer is empty, so the point is considered diverse
    buffer_array = np.array(buffer)
    distances = np.linalg.norm(buffer_array - new_point, axis=1)
    return np.all(distances >= threshold)

def update_buffer(new_point, buffer, buffer_size):
    if len(buffer) >= buffer_size:
        buffer.pop(0)  # Remove the oldest point if the buffer is full
    buffer.append(new_point)

def detect_motion(new_point, last_point, motion_threshold):
    distance = np.linalg.norm(new_point - last_point)
    return distance > motion_threshold

def update_confidence(motion_detected, confidence, max_confidence=100, decrease_factor=5):
    if motion_detected:
        confidence = min(confidence + 1, max_confidence)
    else:
        confidence = max(confidence - decrease_factor, 0)
    return confidence

def remove_outliers_and_calibrate(buffer):
    if len(buffer) == 0:
        return np.array([0.0, 0.0, 0.0]), buffer
    
    data = np.array(buffer)
    mean = np.mean(data, axis=0)
    std_dev = np.std(data, axis=0)
    
    # Remove outliers based on z-score
    z_scores = np.abs((data - mean) / std_dev)
    data_filtered = data[(z_scores < 2).all(axis=1)]
    
    if len(data_filtered) == 0:
        return np.array([0.0, 0.0, 0.0]), data_filtered
    
    # Calculate offsets
    offsets = np.mean(data_filtered, axis=0)
    
    data_corrected = data_filtered - offsets
    
    return offsets, data_corrected

def receive_udp_data():
    global rate, last_point, confidence
    print("UDP thread started.")
    check = "img/"
    check_encoded = check.encode()

    while True:
        data, addr = sock.recvfrom(4096)
        parts = data.split(check_encoded)
        for part in parts:
            if len(part) == 44:
                rate += 1
                values = struct.unpack('<q9f', part)
                _, _, _, _, _, _, _, magX, magY, magZ = values
                if rate % 10 == 0:
                    # magX = magX + 1000
                    # magY = magY
                    # magZ = magZ
                    print( magX, magY, magZ)
                    new_point = np.array([magX, magY, magZ])
                    
                    motion_detected = detect_motion(new_point, last_point, motion_threshold)
                    confidence = update_confidence(motion_detected, confidence)
                    
                    if motion_detected and is_diverse(new_point, buffer, diversity_threshold):
                        update_buffer(new_point, buffer, buffer_size)
                        magnetometer_data.append(new_point)
                        last_point = new_point

                    if confidence >= confidence_threshold:
                        print("Calibration complete with high confidence.")
                        plt.close('all')  # Close the plot when calibration is done
                        return

def update_plot(frame):
    global offsets, magnetometer_data, calibrated_data, buffer
    
    if len(buffer) == 0:
        return
    
    offsets, corrected_data = remove_outliers_and_calibrate(buffer)
    calibrated_data = corrected_data
    
    ax.clear()
    raw_data = np.array(magnetometer_data)
    if len(raw_data) > 0:
        ax.scatter((raw_data[:, 0] - 1550), raw_data[:, 1] - 500, raw_data[:, 2] - 50, c='r', marker='o', label='Raw Data')
    if len(calibrated_data) > 0:
        ax.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2], c='g', marker='^', label='Calibrated Data')
    
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis')
    ax.set_title('Real-Time Magnetometer Data')
    
    # Update the offset text with the latest offsets and confidence level
    offset_text.set_text(f"Offsets - X: {offsets[0]:.2f}, Y: {offsets[1]:.2f}, Z: {offsets[2]:.2f} | Confidence: {confidence}%")
    print(f"Offsets - X: {offsets[0]:.2f}, Y: {offsets[1]:.2f}, Z: {offsets[2]:.2f} | Confidence: {confidence}%")

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
