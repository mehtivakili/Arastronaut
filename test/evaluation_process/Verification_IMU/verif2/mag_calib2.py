import socket
import struct
import numpy as np

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# Initialize min and max values for calibration (start with extreme values)
mag_min = np.array([np.inf, np.inf, np.inf])
mag_max = np.array([-np.inf, -np.inf, -np.inf])

def update_min_max(mag_data, mag_min, mag_max):
    """
    Update the minimum and maximum values for each axis based on the incoming magnetometer data.

    Args:
    mag_data: The raw magnetometer readings (magX, magY, magZ) as a NumPy array.
    mag_min: The minimum values recorded for the magnetometer on each axis.
    mag_max: The maximum values recorded for the magnetometer on each axis.

    Returns:
    Updated min and max values for each axis.
    """
    mag_min = np.minimum(mag_min, mag_data)
    mag_max = np.maximum(mag_max, mag_data)
    
    return mag_min, mag_max

def calculate_offset_scale(mag_min, mag_max):
    """
    Calculate the offset and scale for each axis based on the min and max values.

    Args:
    mag_min: The minimum values recorded for the magnetometer on each axis.
    mag_max: The maximum values recorded for the magnetometer on each axis.

    Returns:
    offset: The offset for each axis.
    scale: The scale for each axis.
    """
    # Calculate the offset for each axis
    offset = (mag_max + mag_min) / 2
    
    # Calculate the scale for each axis
    scale = 2 / (mag_max - mag_min)
    scale = 1 / scale
    
    return offset, scale

def apply_calibration(mag_data, offset, scale):
    """
    Apply the calibration (offset and scale) to the raw magnetometer data.

    Args:
    mag_data: The raw magnetometer readings (magX, magY, magZ) as a NumPy array.
    offset: The offset for each axis.
    scale: The scale for each axis.

    Returns:
    Calibrated magnetometer data as a NumPy array.
    """
    return (mag_data - offset) * scale

rate = 0

# Infinite loop to continuously receive data from the sensor
while True:
    data, addr = sock.recvfrom(4096)
    sock.settimeout(1)

    check = "img/"
    check_encoded = check.encode()
    parts = data.split(check_encoded)

    for part in parts:
        if len(part) == 44:  # 64-bit timestamp + 9 floats
            rate += 1
            if rate == 10:
                rate = 0

                # Unpack the data
                values = struct.unpack('<q9f', part)
                timestamp_ns = values[0]
                timestamp_s = timestamp_ns / 1e9  # Convert nanoseconds to seconds
                accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]

                # Convert magnetometer data to NumPy array
                mag_data = np.array([magX, magY, magZ])

                # Update the min and max values for each axis
                mag_min, mag_max = update_min_max(mag_data, mag_min, mag_max)

                # Calculate the offset and scale
                offset, scale = calculate_offset_scale(mag_min, mag_max)

                # Apply the calibration
                calibrated_mag = apply_calibration(mag_data, offset, scale)

                # Print the results
                print(f"Raw Magnetometer: {mag_data}")
                print(f"Min: {mag_min}, Max: {mag_max}")
                print(f"Offset: {offset}, Scale: {scale}")
                print(f"Calibrated Magnetometer: {calibrated_mag}\n")
