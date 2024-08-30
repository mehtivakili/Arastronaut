import socket
import struct
import numpy as np
import math

rate = 0

# Configuration for the UDP receiver
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 12346  # Port to listen on (must match the ESP32 udpPort)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow the socket to reuse the address
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

# Global variables for data storage
magnetometer_data = []

# Manually set offsets and scales
offsets = np.array([537.0, 214.0, 55.0])  # Example offsets for X, Y, Z axes
scales = np.array([2510.0, 2172.0, 2356.0])  # Example scales for X, Y, Z axes

def calibrate_data(data, offsets, scales):
    # Subtract the offsets and divide by the scales to normalize the data
    return (data - offsets) / scales

def calculate_heading(x, y):
    # Calculate the heading in radians, then convert to degrees
    heading_rad = math.atan2(y, x)
    heading_deg = math.degrees(heading_rad)
    
    # Ensure the heading is in the range [0, 360]
    if heading_deg < 0:
        heading_deg += 360
    
    return heading_deg

def receive_udp_data():
    global rate, magnetometer_data
    
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
                    # magX = magX - 406
                    # magY = magY - 336
                    # magZ = magZ - 38
                    magnetometer_data.append([magX, magY, magZ])
                    
                    # Calibrate the data using the provided offsets and scales
                    raw_data = np.array([magX, magY, magZ])
                    calibrated_data = calibrate_data(raw_data, offsets, scales)
                    
                    # Calculate heading
                    heading = calculate_heading(calibrated_data[0], calibrated_data[1])
                    
                    # Print calibrated data and heading
                    print(f"Calibrated Data - X: {calibrated_data[0]:.2f}, Y: {calibrated_data[1]:.2f}, Z: {calibrated_data[2]:.2f}, Heading: {heading:.2f}°")

# Start the UDP data reception
receive_udp_data()
