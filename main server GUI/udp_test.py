import socket
import struct

# Configuration for the UDP receiver
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 12345  # Port to listen on (must match the ESP32 udpPort)

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on port {UDP_PORT}...")

def receive_udp_data():
    check = "abc/"
    check_encoded = check.encode()

    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        if data.startswith(check_encoded) and len(data) == 28 + len(check_encoded):
            data = data[len(check_encoded):]  # Remove the check variable from the data
            if len(data) == 28:  # Check if the packet size is correct (7 floats * 4 bytes each)
                values = struct.unpack('<7f', data)
                Tio, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = values
                print(f"Tio: {Tio:.2f}, Accel: ({accelX:.2f}, {accelY:.2f}, {accelZ:.2f}), Gyro: ({gyroX:.2f}, {gyroY:.2f}, {gyroZ:.2f})")
            else:
                print(f"Received packet of incorrect size: {len(data)} bytes")
        else:
            print(f"Received corrupted packet from {addr}")

if __name__ == "__main__":
    receive_udp_data()
