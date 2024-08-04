import socket
import struct

UDP_IP = "192.168.4.100"  # Listen on all available interfaces
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def parse_imu_data(data):
    timestamp = struct.unpack('f', data[0:4])[0]
    accelX = struct.unpack('f', data[4:8])[0]
    accelY = struct.unpack('f', data[8:12])[0]
    accelZ = struct.unpack('f', data[12:16])[0]
    gyroX = struct.unpack('f', data[16:20])[0]
    gyroY = struct.unpack('f', data[20:24])[0]
    gyroZ = struct.unpack('f', data[24:28])[0]
    print(f"IMU Data: Time={timestamp}, Accel=({accelX},{accelY},{accelZ}), Gyro=({gyroX},{gyroY},{gyroZ})")

def parse_uwb_data(data):
    tio = struct.unpack('f', data[0:4])[0]
    short_address = struct.unpack('f', data[4:8])[0]
    distance = struct.unpack('f', data[8:12])[0]
    print(f"UWB Data: Time={tio}, Short Address={short_address}, Distance={distance}")


while True:
    data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
    if len(data) > 0:
        if data.startswith(b'abc/'):
            parse_imu_data(data[4:])  # Skip the separator
        elif data.startswith(b'cba/'):
            parse_uwb_data(data[4:])  # Skip the separator
        else:
            print("Unknown data received")

