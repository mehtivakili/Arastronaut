import serial
import struct
import time
import csv

# Serial port configuration
SERIAL_PORT = 'COM6'  # Replace with your actual serial port
BAUD_RATE = 115200  # Must match the baud rate set on the ESP32

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

buffer = b''
expected_interval_ns = 5e6  # Expected interval in nanoseconds (5 ms)
last_timestamp_ns = None  # Last received timestamp from ESP32
pc_last_packet_time_ns = None  # Stores the PC time when the last packet was processed

# CSV setup
output_file = 'imu_data_with_timestamps_nano.csv'
with open(output_file, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Packet Index', 'Timestamp (ns)', 'AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ', 
                         'Timestamp (s)', 'Interval (ms)', 'Latency (ms)'])

    def parse_imu_data(data, packet_index):
        global last_timestamp_ns, pc_last_packet_time_ns
        if len(data) >= 32:
            # Unpack the timestamp and IMU data
            received_timestamp_ns, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = struct.unpack('q6f', data[:32])
            
            if last_timestamp_ns is not None:
                actual_interval_ns = received_timestamp_ns - last_timestamp_ns
                interval_ns = actual_interval_ns
                latency_ns = interval_ns - expected_interval_ns

                # Calculate based on timestamp difference instead of the PC clock
                if interval_ns < expected_interval_ns:
                    latency_ns = 0

            else:
                interval_ns = expected_interval_ns
                latency_ns = 0

            # Update last packet time
            last_timestamp_ns = received_timestamp_ns

            # Convert to human-readable format
            timestamp_s = received_timestamp_ns / 1e9
            interval_ms = interval_ns / 1e6
            latency_ms = latency_ns / 1e6

            # Write data to CSV
            csv_writer.writerow([packet_index, received_timestamp_ns, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, 
                                 f"{timestamp_s:.3f}", f"{interval_ms:.3f}", f"{latency_ms:.3f}"])
            
            # Print the parsed data
            print(f"IMU Data [Packet {packet_index}]: Timestamp={received_timestamp_ns} (s={timestamp_s:.3f}), "
                  f"Accel=({accelX:.3f},{accelY:.3f},{accelZ:.3f}), Gyro=({gyroX:.3f},{gyroY:.3f},{gyroZ:.3f}), "
                  f"Interval={interval_ms:.3f} ms, Latency={latency_ms:.3f} ms")
        else:
            print("Incomplete IMU data received")

    packet_index = 0
    while True:
        if ser.in_waiting > 0:
            buffer += ser.read(1024)  # Read up to 1024 bytes
            while len(buffer) >= 36:
                if buffer.startswith(b'abc/'):
                    parse_imu_data(buffer[4:36], packet_index)
                    buffer = buffer[36:]
                    packet_index += 1
                else:
                    buffer = buffer[1:]
