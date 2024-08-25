import serial
import struct
import time

# Serial port configuration
SERIAL_PORT = 'COM5'  # Replace with your actual serial port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 115200  # Make sure this matches the baud rate set on the ESP32

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

buffer = b''

def parse_imu_data(data):
    if len(data) >= 28:  # Ensure that the data length is correct (7 floats * 4 bytes each)
        timestamp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = struct.unpack('7f', data[:28])
        
        # Calculate the latency
        current_time = time.time()  # Current time in seconds (float)
        latency = current_time - timestamp  # Latency in seconds
        
        # Print the parsed data along with the latency
        print(f"IMU Data: Time={timestamp:.3f}, Accel=({accelX:.3f},{accelY:.3f},{accelZ:.3f}), "
              f"Gyro=({gyroX:.3f},{gyroY:.3f},{gyroZ:.3f})")
    else:
        print("Incomplete IMU data received")

while True:
    if ser.in_waiting > 0:
        buffer += ser.read(1024)  # Read up to 1024 bytes from the serial buffer
        while len(buffer) >= 32:  # 4 bytes for the 'abc/' prefix + 28 bytes for the data
            if buffer.startswith(b'abc/'):
                parse_imu_data(buffer[4:])  # Skip the separator and parse
                buffer = buffer[32:]  # Remove the parsed packet from the buffer
            else:
                print("Unknown data or misalignment detected")
                buffer = buffer[1:]  # Remove the first byte and re-align
