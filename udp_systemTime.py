import socket
import time
import struct

# Set up the UDP socket
udp_port = 12345  # Replace with your actual port number
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', udp_port))

def process_imu_data(data):
    # This function parses the IMU data received from the ESP32
    imu_data = struct.unpack('ffffff', data[:24])  # Assuming 6 floats, 4 bytes each (24 bytes total)
    return imu_data

while True:
    # Receive data from the ESP32
    data, addr = sock.recvfrom(1024)

    # Get the current time in nanoseconds
    current_time_ns = time.time_ns()  # Time in nanoseconds since the epoch
    current_time_sec = current_time_ns / 1_000_000_000  # Convert to seconds

    # Process the received IMU data
    imu_data = process_imu_data(data[8:])  # Assuming the first 8 bytes were for the timestamp

    # Format the output to match the desired format
    output = (
        f"Timestamp={current_time_ns} (s={current_time_sec:.3f}), "
        f"Accel=({imu_data[0]:.3f},{imu_data[1]:.3f},{imu_data[2]:.3f}), "
        f"Gyro=({imu_data[3]:.3f},{imu_data[4]:.3f},{imu_data[5]:.3f})"
    )

    # Print the formatted output
    print(output)

    # Further processing can be done here...
