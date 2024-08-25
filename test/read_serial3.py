import serial
import struct
import time

# Serial port configuration
SERIAL_PORT = 'COM5'  # Replace with your actual serial port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 115200  # Make sure this matches the baud rate set on the ESP32

# Initialize serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

buffer = b''

# Initialize offset value
time_offset = None

def calculate_time_offset():
    """
    Calculate the offset between the IMU time and the Python script time.
    """
    global time_offset

    # Send a synchronization request to the IMU
    sync_request = b'sync_req'  # Define how to request a sync; depends on your IMU's capabilities
    ser.write(sync_request)

    # Measure the time immediately before and after sending the request
    send_time = time.time()

    # Assume the IMU sends back its current timestamp in response
    response = ser.read(1024)
    
    if len(response) >= 4:  # Assuming the response contains at least a float timestamp
        imu_time = struct.unpack('f', response[:4])[0]  # Unpack the IMU's timestamp

        # Measure the time immediately after receiving the response
        receive_time = time.time()

        # Calculate round-trip time
        rtt = receive_time - send_time
        
        # Calculate offset assuming symmetrical delay
        time_offset = (receive_time + send_time) / 2 - imu_time

def parse_imu_data(data):
    if len(data) >= 28:  # Ensure that the data length is correct (7 floats * 4 bytes each)
        timestamp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ = struct.unpack('7f', data[:28])
        
        # Adjust timestamp using the time offset
        if time_offset is not None:
            adjusted_timestamp = timestamp + time_offset
        else:
            adjusted_timestamp = timestamp

        # Calculate the latency
        current_time = time.time()  # Current time in seconds (float)
        latency = current_time - adjusted_timestamp  # Latency in seconds
        
        # Print the parsed data along with the latency
        print(f"IMU Data: Time={timestamp:.3f}, Accel=({accelX:.3f},{accelY:.3f},{accelZ:.3f}), "
              f"Gyro=({gyroX:.3f},{gyroY:.3f},{gyroZ:.3f}), Latency={latency:.6f} seconds")
    else:
        print("Incomplete IMU data received")

# Calculate time offset before entering the main loop
calculate_time_offset()

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
