import serial
import struct
import time
import csv

# Serial port configuration
SERIAL_PORT = 'COM6'  # Replace with your actual serial port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 115200  # Make sure this matches the baud rate set on the ESP32

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

buffer = b''
system_start_time_ns = time.time_ns()  # Get system start time in nanoseconds

# CSV file setup
output_file = 'imu_data_with_timestamps_nano.csv'
with open(output_file, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Timestamp (ns)', 'AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ'])

    def parse_imu_data(data):
        if len(data) >= 32:  # Ensure that the data length is correct (8 bytes for timestamp + 6 floats * 4 bytes each)
            # Unpack the 64-bit timestamp and the 6 floats (AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ)
            received_timestamp_ns, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = struct.unpack('q6f', data[:32])
            
            # Combine the system start time with the received timestamp to get the absolute timestamp
            absolute_timestamp_ns = system_start_time_ns + received_timestamp_ns
            
            # Write to CSV
            csv_writer.writerow([absolute_timestamp_ns, accelX, accelY, accelZ, gyroX, gyroY, gyroZ])
            
            # Print the parsed data along with the calculated timestamp
            print(f"IMU Data: Timestamp={absolute_timestamp_ns}, Accel=({accelX:.3f},{accelY:.3f},{accelZ:.3f}), "
                  f"Gyro=({gyroX:.3f},{gyroY:.3f},{gyroZ:.3f})")
        else:
            print("Incomplete IMU data received")

    while True:
        if ser.in_waiting > 0:
            buffer += ser.read(1024)  # Read up to 1024 bytes from the serial buffer
            while len(buffer) >= 36:  # 4 bytes for the 'abc/' prefix + 32 bytes for the data
                if buffer.startswith(b'abc/'):
                    parse_imu_data(buffer[4:36])  # Skip the separator and parse the next 32 bytes
                    buffer = buffer[36:]  # Remove the parsed packet from the buffer
                else:
                    print("Unknown data or misalignment detected")
                    buffer = buffer[1:]  # Remove the first byte and re-align
