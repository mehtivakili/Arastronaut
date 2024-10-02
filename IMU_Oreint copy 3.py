import numpy as np
import socket
import struct
import threading
import queue
import matplotlib.pyplot as plt
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import time
import serial

# Constants
MAG_LOWER_BOUND = -100000
MAG_UPPER_BOUND = 100000

# Helper functions
def filter_magnetometer(mag, lower_bound, upper_bound):
    """Filter magnetometer data based on threshold."""
    return np.all((mag >= lower_bound) & (mag <= upper_bound))

def load_calibration(file):
    """Load IMU calibration data from files."""
    with open(file, 'r') as f:
        lines = f.readlines()
    # Parse misalignment matrix
    misalignment = np.array([[float(x) for x in lines[i].split()] for i in range(3)])
    # Parse scale matrix
    scale = np.array([[float(x) for x in lines[i].split()] for i in range(4, 7)])
    # Parse bias vector
    bias = np.array([float(lines[i].split()[0]) for i in range(8, 11)])
    return misalignment, scale, bias

def apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias):
    """Apply calibration to accelerometer and gyroscope data."""
    accel_corrected = np.dot(acc_misalignment, accel - acc_bias)
    accel_calibrated = np.dot(acc_scale, accel_corrected)
    gyro_corrected = np.dot(gyro_misalignment, gyro - gyro_bias)
    gyro_calibrated = np.dot(gyro_scale, gyro_corrected)
    return accel_calibrated, gyro_calibrated

def calibrate_magnetometer(data, offsets, scales):
    """Calibrate and scale magnetometer data."""
    return np.dot(data - offsets, scales)

def quaternion_to_axis_angle(q):
    """Convert quaternion [w, x, y, z] to axis-angle representation."""
    r = R.from_quat([q[1], q[2], q[3], q[0]])  # [x, y, z, w] format
    angle = r.magnitude()  # Total rotation angle
    axis = r.as_rotvec() / angle  # Rotation axis
    return axis, np.degrees(angle)

def angular_difference(angle1, angle2):
    diff = angle1 - angle2
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff

def initialize_orientation(accel, mag):
    """Initialize orientation using accelerometer and magnetometer."""
    # Normalize accelerometer and magnetometer readings
    accel_norm = accel / np.linalg.norm(accel)
    mag_norm = mag / np.linalg.norm(mag)
    
    # Calculate pitch and roll from accelerometer
    pitch = np.arcsin(-accel_norm[0])
    roll = np.arctan2(accel_norm[1], accel_norm[2])
    
    # Adjust magnetometer by pitch and roll to get correct yaw
    mag_x = (mag_norm[0] * np.cos(pitch) + mag_norm[2] * np.sin(pitch))
    mag_y = (mag_norm[0] * np.sin(roll) * np.sin(pitch) +
             mag_norm[1] * np.cos(roll) -
             mag_norm[2] * np.sin(roll) * np.cos(pitch))
    
    yaw = np.arctan2(-mag_y, mag_x)
    
    # Convert to quaternion
    initial_quaternion = euler2quat(roll, pitch, yaw)
    return initial_quaternion

def euler2quat(roll, pitch, yaw):
    """
    Convert roll, pitch, yaw (in radians) to quaternion [w, x, y, z].
    """
    r = R.from_euler('xyz', [roll, pitch, yaw])  # Use scipy's Rotation to convert
    q = r.as_quat()  # Returns [x, y, z, w] quaternion
    return np.array([q[3], q[0], q[1], q[2]])  # Rearrange to [w, x, y, z] format

# Thread to collect encoder data
def serial_thread(serial_port, encoder_queue):
    ser = serial.Serial(serial_port, 115200, timeout=0.01)
    while True:
        try:
            line = ser.readline().decode().strip()
            if line:
                position = int(line.split(':')[-1].strip())
                print(position)

                encoder_queue.put(position)
        except (serial.SerialException, ValueError):
            pass

# Data processing function
def data_thread(sock, data_queue, encoder_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales):
    madgwick = Madgwick(beta=0.6, frequency=200)
    q = None
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            parts = data.split(b'img/')
            for part in parts:
                if len(part) == 44:
                    values = struct.unpack('<q9f', part)
                    accel = np.array(values[1:4])
                    gyro = np.array(values[4:7])
                    mag = np.array(values[7:10])

                    accel, gyro = apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias)
                    mag = calibrate_magnetometer(mag, mag_offsets, mag_scales)

                    # Normalizing the magnetometer data
                    mag = mag / np.linalg.norm(mag)

                    # Calculate heading from magnetometer
                    heading_mag = np.arctan2(mag[1], mag[0])
                    # Convert heading to degrees
                    heading_degrees = np.degrees(heading_mag)
                    if heading_degrees < 0:
                        heading_degrees += 360

                    print(f"Heading Mag: {heading_degrees} degrees")
                    # Normalize accelerometer and magnetometer data
                    accel = accel / np.linalg.norm(accel)
                    mag = mag / np.linalg.norm(mag)
                    
                    # Calculate roll and pitch from accelerometer data
                    roll = np.arctan2(accel[1], accel[2])
                    pitch = np.arctan2(-accel[0], np.sqrt(accel[1]**2 + accel[2]**2))
                    
                    # Tilt compensation for the magnetometer data
                    mag_x = mag[0] * np.cos(pitch) + mag[2] * np.sin(pitch)
                    mag_y = mag[0] * np.sin(roll) * np.sin(pitch) + mag[1] * np.cos(roll) - mag[2] * np.sin(roll) * np.cos(pitch)
                    
                    # Calculate heading using the tilt-compensated magnetometer data
                    heading_fuse = np.arctan2(-mag_y, mag_x)

                    # Convert heading to degrees
                    heading_degrees = np.degrees(heading_fuse)
                    if heading_degrees < 0:
                        heading_degrees += 360

                    print(f"Heading fuse: {heading_degrees} degrees")
                    
                    if q is None:
                        q = initialize_orientation(accel, mag)
                    else:
                        q = madgwick.updateMARG(q=q, gyr=gyro, acc=accel, mag=mag)

                    # Calculate yaw from angle-axis representation
                    axis, angle = quaternion_to_axis_angle(q)
                    yaw_imu_deg = angle if axis[2] >= 0 else -angle  # Use the Z-axis component for yaw

                    # Get encoder data
                    if not encoder_queue.empty():
                        yaw_encoder = encoder_queue.get()
                        data_queue.put((yaw_imu_deg, yaw_encoder))

        except socket.timeout:
            continue
        except KeyboardInterrupt:
            break

# Visualization function
def plot_thread(data_queue):
    plt.ion()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

    yaw_encoder_vals = []
    yaw_imu_vals = []
    yaw_error_vals = []
    time_vals = []

    line1, = ax1.plot([], [], label="Encoder Yaw (°)", color="blue")
    line2, = ax1.plot([], [], label="IMU Yaw (°)", color="green")
    line3, = ax2.plot([], [], label="Yaw Error (°)", color="red")

    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Yaw Angle (°)')
    ax1.legend()

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (°)')
    ax2.legend()

    start_time = time.time()

    while True:
        if not data_queue.empty():
            yaw_imu_deg, yaw_encoder = data_queue.get()

            elapsed_time = time.time() - start_time
            error = angular_difference(yaw_imu_deg, yaw_encoder)

            time_vals.append(elapsed_time)
            yaw_encoder_vals.append(yaw_encoder)
            yaw_imu_vals.append(yaw_imu_deg)
            yaw_error_vals.append(error)

            # Update plots
            line1.set_xdata(time_vals)
            line1.set_ydata(yaw_encoder_vals)

            line2.set_xdata(time_vals)
            line2.set_ydata(yaw_imu_vals)

            line3.set_xdata(time_vals)
            line3.set_ydata(yaw_error_vals)

            # Rescale plots
            ax1.relim()
            ax1.autoscale_view()
            ax2.relim()
            ax2.autoscale_view()

            plt.draw()
            plt.pause(0.01)

# Main setup
if __name__ == "__main__":
    UDP_IP = "0.0.0.0"
    UDP_PORT = 12346
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))

    serial_port = "COM7"  # Replace with the actual serial port
    acc_misalignment, acc_scale, acc_bias = load_calibration('./calib_data/test_imu_acc12G.calib')
    gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./calib_data/test_imu_gyro12G.calib')
    mag_offsets = np.array([1463.4, 2316.1, -382.8])
    mag_scales = np.array([[1.0207, 0.00315, 0.0538], [-0.0315, 0.9730, 0.0980], [0.0538, 0.0980, 1.0209]])

    data_queue = queue.Queue()
    encoder_queue = queue.Queue()

    data_thread = threading.Thread(target=data_thread, args=(sock, data_queue, encoder_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales))
    data_thread.daemon = True
    data_thread.start()

    serial_thread = threading.Thread(target=serial_thread, args=(serial_port, encoder_queue))
    serial_thread.daemon = True
    serial_thread.start()

    plot_thread(data_queue)
