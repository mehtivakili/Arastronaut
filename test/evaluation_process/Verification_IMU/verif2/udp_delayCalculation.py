import socket
import struct
import csv

# UDP configuration
UDP_IP = "192.168.4.100"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

buffer = b''
expected_interval_ns = 5e6  # Expected interval between packets in nanoseconds (5 ms)
last_timestamp_ns = None
pc_last_packet_time_ns = None

# CSV setup
output_file = 'udp_data_with_timestamps_nano.csv'
with open(output_file, 'w', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    csv_writer.writerow(['Packet Index', 'Timestamp (ns)', 'AccelX', 'AccelY', 'AccelZ', 'GyroX', 'GyroY', 'GyroZ',
                         'Timestamp (s)', 'Interval (ms)', 'Latency (ms)'])

    def parse_udp_data(data, packet_index):
        global last_timestamp_ns, pc_last_packet_time_ns
        if len(data) >= 32:
            received_timestamp_ns, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = struct.unpack('q6f', data[:32])

            if last_timestamp_ns is not None:
                if received_timestamp_ns == last_timestamp_ns:
                    print(f"Duplicate packet detected at index {packet_index}, ignoring...")
                    return

                actual_interval_ns = received_timestamp_ns - last_timestamp_ns
                interval_ns = actual_interval_ns
                latency_ns = interval_ns - expected_interval_ns

                if interval_ns < expected_interval_ns:
                    latency_ns = 0
            else:
                interval_ns = expected_interval_ns
                latency_ns = 0

            last_timestamp_ns = received_timestamp_ns

            timestamp_s = received_timestamp_ns / 1e9
            interval_ms = interval_ns / 1e6
            latency_ms = latency_ns / 1e6

            csv_writer.writerow([packet_index, received_timestamp_ns, accelX, accelY, accelZ, gyroX, gyroY, gyroZ,
                                 f"{timestamp_s:.3f}", f"{interval_ms:.3f}", f"{latency_ms:.3f}"])

            print(f"UDP Data [Packet {packet_index}]: Timestamp={received_timestamp_ns} (s={timestamp_s:.3f}), "
                  f"Accel=({accelX:.3f},{accelY:.3f},{accelZ:.3f}), "
                  f"Gyro=({gyroX:.3f},{gyroY:.3f},{gyroZ:.3f}), Interval={interval_ms:.3f} ms, "
                  f"Latency={latency_ms:.3f} ms")
        else:
            print("Incomplete UDP data received")

    packet_index = 0
    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size is 1024 bytes
        if len(data) > 0:
            if data.startswith(b'abc/'):
                parse_udp_data(data[4:36], packet_index)
                packet_index += 1
            else:
                print("Unknown data or misalignment detected")
