import numpy as np
import socket
import struct
import threading
import queue
import time
import serial
from ahrs.filters import Madgwick
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import traceback
import csv

# Encoder parameters
PULSES_PER_REV = 583  # Define the number of pulses per revolution for the encoder
initial_angle_offset = 0.0  # Set initial position angle (e.g., starting at -150°)
invert_direction = True  # Set to True to invert encoder direction
prev_position = 0  # Previous encoder position
prev_time = 0  # Previous timestamp

def calculate_angle_omega(position, current_time):
    global prev_position, prev_time, initial_angle_offset
    
    # Ensure initial_angle_offset is set before proceeding
    if initial_angle_offset is None:
        print("Waiting for initial_angle_offset to be set...")
        return None, None  # Return None to indicate no valid angle yet
    
    # Invert the encoder position if needed
    if invert_direction:
        position = -position

    # Calculate the raw angle of rotation in degrees based on the encoder position
    raw_angle = (position % PULSES_PER_REV) * (360.0 / PULSES_PER_REV)
    
    # Adjust the raw angle to be between -180 and 180 degrees
    if raw_angle > 180:
        raw_angle -= 360  # Wrap angle to stay between -180° and 180°
    
    # Apply the initial offset (starting angle)
    angle = raw_angle + initial_angle_offset
    
    # Ensure the angle is still wrapped within -180° to 180°
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360
    
    # Calculate angular velocity (omega)
    time_diff = current_time - prev_time
    position_diff = position - prev_position
    
    # Calculate omega (angular velocity) in degrees per second
    if time_diff > 0:
        omega = (position_diff * (360.0 / PULSES_PER_REV)) / time_diff
    else:
        omega = 0

    # Update previous values
    prev_position = position
    prev_time = current_time
    
    return angle, omega

# Function to apply calibration to accelerometer and gyroscope data
def apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias):
    accel_corrected = np.dot(acc_misalignment, accel - acc_bias)
    accel_calibrated = np.dot(acc_scale, accel_corrected)
    
    gyro_corrected = np.dot(gyro_misalignment, gyro - gyro_bias)
    gyro_calibrated = np.dot(gyro_scale, gyro_corrected)
    
    return accel_calibrated, gyro_calibrated

# Function to calibrate magnetometer data
def calibrate_data(data, offsets, scales):
    return (data - offsets) / scales

# Initialize orientation using accelerometer and magnetometer
def initialize_orientation(accel, mag):
    accel_norm = accel / np.linalg.norm(accel)
    mag_norm = mag / np.linalg.norm(mag)
    
    pitch = np.arctan2(accel_norm[1], accel_norm[2])
    roll = np.arctan2(-accel_norm[0], np.sqrt(accel_norm[1]**2 + accel_norm[2]**2))
    
    mag_x = mag_norm[0] * np.cos(pitch) + mag_norm[2] * np.sin(pitch)
    mag_y = mag_norm[0] * np.sin(roll) * np.sin(pitch) + mag_norm[1] * np.cos(roll) - mag_norm[2] * np.sin(roll) * np.cos(pitch)
    yaw = np.arctan2(-mag_y, mag_x)  # Yaw angle (heading) from magnetometer
    
    initial_quaternion = euler2quat(roll, pitch, yaw)
    return initial_quaternion

# Load calibration data
def load_calibration(file):
    with open(file, 'r') as f:
        lines = f.readlines()

    misalignment = np.array([
        [float(x) for x in lines[0].split()],
        [float(x) for x in lines[1].split()],
        [float(x) for x in lines[2].split()]
    ])
    scale = np.array([
        [float(x) for x in lines[4].split()],
        [float(x) for x in lines[5].split()],
        [float(x) for x in lines[6].split()]
    ])
    bias = np.array([
        float(lines[8].split()[0]),
        float(lines[9].split()[0]),
        float(lines[10].split()[0])
    ])

    return misalignment, scale, bias

# Convert Euler angles to quaternion
def euler2quat(roll, pitch, yaw):
    r = R.from_euler('xyz', [roll, pitch, yaw])
    return r.as_quat()  # Returns [x, y, z, w] quaternion

# Initialize the Madgwick filter and variables
madgwick = Madgwick(beta=0.2)
q = np.array([1.0, 0.0, 0.0, 0.0])
init_rot = True
initial_angle_offset = None  # This will be set after 5 seconds

# Function to process IMU data and update the Madgwick filter
start_time = time.time()  # Record the start time

def data_thread(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales):
    previous_timestamp_s = 0.0
    rate = 0
    rate2 = 0
    global init_rot, initial_angle_offset
    initial_angle_offset_isnot_set = True
    first_init = True
    while True:
        try:
            data, addr = sock.recvfrom(4096)
            check = "img/"
            check_encoded = check.encode()
            parts = data.split(check_encoded)

            for part in parts:
                if len(part) == 44:
                    rate += 1
                    if rate == 1:
                        rate = 0
                        values = struct.unpack('<q9f', part)
                        timestamp_ns = values[0]
                        timestamp_s = timestamp_ns / 1e9
                        accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]

                        accel = np.array([accelX, accelY, accelZ])
                        gyro = np.array([gyroX, gyroY, gyroZ])
                        mag = np.array([magX, magY, magZ])

                        accel, gyro = apply_calibration(accel, gyro, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias)
                        mag = calibrate_data(mag, mag_offsets, mag_scales)

                        if init_rot:
                            initial_orientation = initialize_orientation(accel, mag)
                            q = np.array([initial_orientation[3], initial_orientation[0], initial_orientation[1], initial_orientation[2]])
                            
                            init_rot = False

                        q = madgwick.updateMARG(q=q, gyr=gyro, acc=accel, mag=mag)
                        pitch = np.arctan2(2.0 * (q[3] * q[2] + q[0] * q[1]), q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)
                        roll = np.arcsin(-2.0 * (q[1] * q[3] - q[0] * q[2]))
                        yaw = np.arctan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2)

                        # Convert to degrees
                        yaw = np.degrees(yaw)
                        pitch = np.degrees(pitch)
                        roll = np.degrees(roll)
                        
                        if first_init :
                            initial_angle_offset = np.degrees(yaw)
                            first_init = False

                        
                        rate2 = rate2 + 1
                        if rate2 == 20:
                            data_queue.put(q)

                        #     print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")
                        #     print(f"Quaternion: {q}")
                        #     rate2 = 0
                        # Check if 5 seconds have passed since the start of the thread
                        if initial_angle_offset_isnot_set and (time.time() - start_time) >= 5:
                            yaw_angle = np.arctan2(2.0 * (q[1] * q[2] + q[0] * q[3]), q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2)  ########  Yaw
                            # yaw_angle = np.arcsin(-2.0 * (q[1] * q[3] - q[0] * q[2]))  ############  Roll
                            # yaw_angle = np.arctan2(2.0 * (q[3] * q[2] + q[0] * q[1]), q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2) ########   Pitch
                          
                            initial_angle_offset = np.degrees(yaw_angle)
                            print(f"Initial angle offset set to: {initial_angle_offset} degrees")
                            initial_angle_offset_isnot_set = False

                        # data_queue.put(q)
        except Exception as e:
            print("Exception in data_thread:", e)
            traceback.print_exc()

# Thread to read serial data from the encoder
def serial_thread(serial_port, encoder_queue):
    global initial_angle_offset
    last_encoder_value = 0
    enc = 0
    start_time = time.time()
    print("Starting serial_thread...")
    try:
        ser = serial.Serial(serial_port, 915200, timeout=0.5)
        while True:
            encoder_queue.put(enc)
            # print(initial_angle_offset)
            if (time.time() - start_time) >= 6:
                calculate_angle_omega(0,0)
                # print("done")
                # print(time.time())
            try:
                line = ser.readline().decode().strip()
                if line:
                    position = int(line.split(':')[-1].strip())
                    current_time = time.time()

                    angle, omega = calculate_angle_omega(position, current_time)
                    
                    enc = angle
                    encoder_queue.put(enc)      
                #     if angle is not None:
                #         # Update the last known encoder value
                #         last_encoder_value = angle

                #         # Put the calculated angle in the encoder queue
                #         encoder_queue.put(angle)
                # else:
                #     # If no new data, resend the last known encoder value if available
                #     if last_encoder_value is not None:
                #         encoder_queue.put(last_encoder_value)
                    # print(angle)
                    # encoder_queue.put(angle)
                # time.sleep(0.1)

            except serial.SerialException as e:
                print("Serial read error!", e)
            except ValueError:
                print("Invalid encoder data!")
    except Exception as e:
        print("Exception in serial_thread:", e)
        traceback.print_exc()
        
# Function to calculate the angular difference considering wrap-around at 180°
def angular_difference(angle1, angle2):
    diff = angle1 - angle2
    # Wrap the difference to be between -180 and 180
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff

# Function to wrap angle in the range -180° to 180°
def wrap_to_180(angle):
    # Use modulo to wrap the angle within 360 degrees
    wrapped_angle = (angle + 180) % 360 - 180
    return wrapped_angle


def quaternion_to_euler(q):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).
    
    Args:
        q: A quaternion in the form [qw, qx, qy, qz]
    
    Returns:
        A tuple of Euler angles (roll, pitch, yaw) in radians.
    """
    qw, qx, qy, qz = q
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx ** 2 + qy ** 2)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = np.pi / 2 * np.sign(sinp)  # use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy ** 2 + qz ** 2)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return roll, pitch, yaw

# Function to collect and compute yaw values in the background thread
def error_thread(data_queue, encoder_queue, plot_queue, csv_file_path):
    print("Starting error_thread...")
    global initial_angle_offset
    
    start_time = time.time()
    delay = 6  # Delay in seconds
    init = True
    
    # Open CSV file for writing
    file = None
    try:
        file = open("Yaw Error.csv", mode='w', newline='')  # Open the file for writing
        writer = csv.writer(file)
        # Write header with timestamp, encoder data, error, roll, pitch, and yaw
        writer.writerow(["Timestamp", "Encoder Data", "Error", "Roll (deg)", "Pitch (deg)", "Yaw (deg)"])  
        file.flush()  # Force write to file
        
        while True:
            try:
                # Get data from queues
                q = data_queue.get(timeout=0.1)  # Wait for 100ms for new data

                # if q is None:
                #     print("No data in data_queue.")
                #     continue

                # Calculate roll, pitch, and yaw from quaternion
                euler_angles = quaternion_to_euler(q)
                roll, pitch, yaw = euler_angles
                
                # Convert angles to degrees
                roll = np.degrees(roll)
                pitch = np.degrees(pitch)
                yaw = np.degrees(yaw)

                # Get encoder data
                yaw_encoder = encoder_queue.get(timeout=0.2)

                if yaw_encoder is None:
                    # print("No data in encoder_queue.")
                    continue

                # Calculate yaw error
                error = angular_difference(yaw, yaw_encoder)

                current_time = time.time()  # Get current time

                # Skip initial condition where encoder is zero
                if yaw_encoder != 0:
                    init = False

                if yaw_encoder == 0 and init:
                    yaw_encoder = initial_angle_offset

                # Every 100 ms, write the data to CSV
                timestamp = current_time - start_time
                writer.writerow([timestamp, yaw_encoder, error, roll, pitch, yaw])  # Write data row
                file.flush()  # Force write to file to ensure it's not being buffered

                print(f"Written to CSV - Timestamp: {timestamp:.2f}s, Encoder: {yaw_encoder:.2f}°, Error: {error:.2f}°, Roll: {roll:.2f}°, Pitch: {pitch:.2f}°, Yaw: {yaw:.2f}°")
                
                # time.sleep(0.1)  # Sleep for 100ms (10Hz)

            except queue.Empty:
                # print("Queue is empty.")
                pass  # Continue if the queue is empty

    except Exception as e:
        print("Exception in error_thread:", e)
        traceback.print_exc()
    
    finally:
        # Ensure the file is closed properly
        if file is not None:
            file.close()
            print("CSV file closed properly.")        
# Function to handle real-time plotting (must be run in the main thread)
def plot_thread(plot_queue):
    print("Starting plot_thread...")

    # Lists to store values for plotting
    yaw_encoder_vals = []
    yaw_madgwick_vals = []
    yaw_error_vals = []
    time_vals = []

    # Initialize plot
    plt.ion()  # Turn on interactive mode for real-time updates
    fig, (ax1, ax2) = plt.subplots(2, 1)  # Two subplots: ax1 for yaw, ax2 for error
    
    # Plot for Yaw Encoder and Madgwick
    line1, = ax1.plot([], [], label="Yaw Encoder (°)", color='blue')
    line2, = ax1.plot([], [], label="Yaw Madgwick (°)", color='green')
    ax1.legend()
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Degrees (°)')
    
    # Plot for Yaw Error
    line3, = ax2.plot([], [], label="Yaw Error (°)", color='red')
    ax2.legend()
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error (°)')
    
    # Text placeholders for displaying values
    yaw_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=10, verticalalignment='top')
    error_text = ax2.text(0.02, 0.95, '', transform=ax2.transAxes, fontsize=10, verticalalignment='top')

    start_time = time.time()
    plot_delay = 6  # Delay in seconds
    
    while True:
        # Get the current time to check if the plot should start
        current_time = time.time()

        # Check if 6 seconds have passed
        if current_time - start_time >= plot_delay:
            # Get data from the plot_queue
            # Get data from the plot_queue
            if not plot_queue.empty():
                yaw_encoder, yaw_madgwick_deg, error, current_time = plot_queue.get()

                # Update data lists for plotting
                elapsed_time = current_time - start_time
                time_vals.append(elapsed_time)
                yaw_encoder_vals.append(yaw_encoder)
                yaw_madgwick_vals.append(yaw_madgwick_deg)
                yaw_error_vals.append(error)

                # Update the plot data for Yaw Encoder and Madgwick
                line1.set_xdata(time_vals)
                line1.set_ydata(yaw_encoder_vals)
                line2.set_xdata(time_vals)
                line2.set_ydata(yaw_madgwick_vals)

                # Update the plot data for Yaw Error
                line3.set_xdata(time_vals)
                line3.set_ydata(yaw_error_vals)

                # Update the text on the plot with the latest values
                yaw_text.set_text(f'Yaw Encoder: {yaw_encoder:.2f}°, Yaw Madgwick: {yaw_madgwick_deg:.2f}°')
                error_text.set_text(f'Yaw Error: {error:.2f}°')

                # Recompute limits and rescale
                ax1.relim()  
                ax1.autoscale_view()  
                ax2.relim()  
                ax2.autoscale_view()

                # Draw the updated plot
                plt.draw()
                plt.pause(0.01)  # Pause to update the plot

        
        
# Main logic to start threads
if __name__ == "__main__":
    print("Starting main program...")

    # Setup UDP and Serial connections
    UDP_IP = "0.0.0.0"
    UDP_PORT = 12346
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT))

    serial_port = "COM4"  # Set this to the correct serial port for your encoder

    # Load calibration data
    acc_misalignment, acc_scale, acc_bias = load_calibration('./main server GUI/calib_data/test_imu_acc6G.calib')
    gyro_misalignment, gyro_scale, gyro_bias = load_calibration('./main server GUI/calib_data/test_imu_gyro.calib')
    mag_offsets = np.array([86.0, 141.0, -105.0])
    mag_scales = np.array([2284.0, 2332.0, 2312.0])
    
    # Define the path where you want to save the CSV file
    csv_file_path = "output.csv"
    
    # Queues for passing data
    data_queue = queue.Queue()
    encoder_queue = queue.Queue()
    plot_queue = queue.Queue()  # Queue to send data for plotting

    # Start threads
    data_thread = threading.Thread(target=data_thread, args=(sock, data_queue, acc_misalignment, acc_scale, acc_bias, gyro_misalignment, gyro_scale, gyro_bias, mag_offsets, mag_scales))
    data_thread.daemon = True
    data_thread.start()

    serial_thread = threading.Thread(target=serial_thread, args=(serial_port, encoder_queue))
    serial_thread.daemon = True
    serial_thread.start()

    error_thread = threading.Thread(target=error_thread, args=(data_queue, encoder_queue, plot_queue, csv_file_path))
    error_thread.daemon = True
    error_thread.start()
    
    # Start the plot_thread to handle plotting in the main thread
    plot_thread(plot_queue)
    

    # Keep the main program running
    while True:
        time.sleep(0.1)
        # print("yes")