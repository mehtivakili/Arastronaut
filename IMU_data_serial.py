import serial
import struct
import time
import csv

# Serial port configuration
SERIAL_PORT = 'COM5'  # Replace with your actual serial port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 115200  # Make sure this matches the baud rate set on the ESP32

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

buffer = b''
initial_time_ns = time.time_ns()  # Get initial time in nanoseconds

# CSV file setup
output_file = 'imu_data_with_timestamps.csv'
with open(output_file, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Timestamp (ns)', 'AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ'])

    def parse_imu_data(data):
        if len(data) >= 28:  # Ensure that the data length is correct (7 floats * 4 bytes each)
            timestamp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = struct.unpack('7f', data[:28])
            
            # Calculate the actual timestamp in nanoseconds
            timestamp_ns = initial_time_ns + int(timestamp * 1e9)
            
            # Write to CSV
            csv_writer.writerow([timestamp_ns, accelX, accelY, accelZ, gyroX, gyroY, gyroZ])
            
            # Print the parsed data along with the calculated timestamp
            print(f"IMU Data: Timestamp={timestamp_ns}, Accel=({accelX:.3f},{accelY:.3f},{accelZ:.3f}), "
                  f"Gyro=({gyroX:.3f},{gyroY:.3f},{gyroZ:.3f})")
        else:
            print("Incomplete IMU data received")

    while True:
        if ser.in_waiting > 0:
            buffer += ser.read(1024)  # Read up to 1024 bytes from the serial buffer
            while len(buffer) >= 32:  # 4 bytes for the 'abc/' prefix + 28 bytes for the data
                if buffer.startswith(b'abc/'):
                    parse_imu_data(buffer[4:32])  # Skip the separator and parse the next 28 bytes
                    buffer = buffer[32:]  # Remove the parsed packet from the buffer
                else:
                    print("Unknown data or misalignment detected")
                    buffer = buffer[1:]  # Remove the first byte and re-align
