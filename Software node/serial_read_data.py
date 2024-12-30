import serial
import struct

# Serial port configuration
SERIAL_PORT = 'COM5'  # Replace with your actual serial port (e.g., COM3 on Windows)
BAUD_RATE = 115200  # Set the baud rate to 915200

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def parse_imu_mag_data(data):
    timestamp = struct.unpack('f', data[0:4])[0]
    accelX = struct.unpack('f', data[4:8])[0]
    accelY = struct.unpack('f', data[8:12])[0]
    accelZ = struct.unpack('f', data[12:16])[0]
    gyroX = struct.unpack('f', data[16:20])[0]
    gyroY = struct.unpack('f', data[20:24])[0]
    gyroZ = struct.unpack('f', data[24:28])[0]
    magX = struct.unpack('f', data[28:32])[0]
    magY = struct.unpack('f', data[32:36])[0]
    magZ = struct.unpack('f', data[36:40])[0]

    print(f"IMU Data: Time={timestamp}, Accel=({accelX},{accelY},{accelZ}), Gyro=({gyroX},{gyroY},{gyroZ}), Mag=({magX},{magY},{magZ})")

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
    time = struct.unpack('f', data[0:4])[0]
    address = struct.unpack('f', data[4:8])[0]
    distance = struct.unpack('f', data[8:12])[0]
    print(f"UWB Data: Time = {time} , address = {address}, Distance={distance}")

def parse_mag_data(data):
    time = struct.unpack('f', data[0:4])[0]
    magX = struct.unpack('f', data[4:8])[0]
    magY = struct.unpack('f', data[8:12])[0]
    magZ = struct.unpack('f', data[12:16])[0]

    print(f"Mag Data: Time={time}  Mag=({magX},{magY},{magZ})")

while True:
    if ser.in_waiting > 0:
        data = ser.read(1024)  # Read up to 1024 bytes from the serial buffer
        print(f"Raw data received: {data}")  # Print the raw data for debugging

        if len(data) > 0:
            if data.startswith(b'img/'):
                parse_imu_mag_data(data[4:])  # Skip the separator
            elif data.startswith(b'cba/'):
                parse_uwb_data(data[4:])  # Skip the separator
            elif data.startswith(b'mag'):
                parse_mag_data(data[4:])
            elif data.startswith(b'abc/'):
                parse_imu_data(data[4:])
            else:
                print("Unknown data received")
