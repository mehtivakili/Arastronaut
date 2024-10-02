import socket
import struct
import numpy as np
import threading
import queue
import allantools
import matplotlib.pyplot as plt
import time
import pandas as pd
from scipy.interpolate import interp1d

# UDP setup
UDP_IP = "0.0.0.0"
UDP_PORT = 12346
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))

# Thread-safe queue to store gyroscope data
gyro_queue = queue.Queue()

def data_thread(sock, gyro_queue):
    """
    Thread to receive data from UDP and extract gyroscope readings.
    """
    check_encoded = "img/".encode()
    
    while True:
        try:
            # Receive data from UDP
            data, addr = sock.recvfrom(4096)
            # Split data based on the specific header
            parts = data.split(check_encoded)
            for part in parts:
                if len(part) == 44:  # 8 bytes timestamp + 9*4 bytes floats
                    # Unpack the data
                    values = struct.unpack('<q9f', part)
                    timestamp_ns = values[0]
                    timestamp_s = timestamp_ns / 1e9  # Convert nanoseconds to seconds
                    accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ = values[1:]
                    
                    # Append gyroscope data as a tuple (timestamp, gyroX, gyroY, gyroZ)
                    gyro_queue.put((timestamp_s, gyroX, gyroY, gyroZ))
                    
        except Exception as e:
            print(f"Error receiving data: {e}")

# Start the data receiving thread
thread = threading.Thread(target=data_thread, args=(sock, gyro_queue), daemon=True)
thread.start()

def collect_gyro_data(gyro_queue, duration=60):
    """
    Collect gyroscope data for a specified duration (in seconds).
    """
    print(f"Collecting gyroscope data for {duration} seconds...")
    start_time = time.time()
    gyro_data = []
    
    while time.time() - start_time < duration:
        try:
            # Wait for data with a timeout to prevent blocking indefinitely
            data = gyro_queue.get(timeout=1)
            timestamp, gx, gy, gz = data
            gyro_data.append([timestamp, gx, gy, gz])
        except queue.Empty:
            print("No data received in the last second.")
            continue
    
    print(f"Collected {len(gyro_data)} samples.")
    return np.array(gyro_data)

def resample_data(timestamps, data, fs):
    """
    Resample data to regular intervals using scipy.interpolate.interp1d.
    """
    # Create regular timestamps based on desired sampling frequency
    regular_timestamps = np.arange(timestamps[0], timestamps[-1], 1.0/fs)
    
    # Ensure that there are enough points for interpolation
    if len(timestamps) < 2:
        raise ValueError("Not enough data points for interpolation.")
    
    # Create interpolation function
    interp_func = interp1d(timestamps, data, kind='linear', fill_value="extrapolate")
    
    # Perform interpolation
    resampled_data = interp_func(regular_timestamps)
    
    return regular_timestamps, resampled_data

# Example usage: Collect data for 60 seconds
if __name__ == "__main__":
    duration = 200  # seconds
    gyro_data = collect_gyro_data(gyro_queue, duration=duration)
    
    # Check if data was collected
    if gyro_data.size == 0:
        print("No gyroscope data collected.")
        exit(1)
    
    # Extract gyro axes
    timestamps = gyro_data[:, 0]
    gyro_x = gyro_data[:, 1]
    gyro_y = gyro_data[:, 2]
    gyro_z = gyro_data[:, 3]
    
    # Check statistics of gyro_x before resampling
    print(f"Gyro X-axis - min: {gyro_x.min()}, max: {gyro_x.max()}, mean: {gyro_x.mean()}, std: {gyro_x.std()}")
    
    # Optionally, compute the magnitude of gyroscope vector
    gyro_mag = np.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
    
    # Choose which axis or magnitude to analyze
    # For example, analyze gyro_x
    data_to_analyze = gyro_x  # Replace with gyro_y, gyro_z, or gyro_mag as needed
    
    # Ensure data is sampled at regular intervals
    # Compute sampling frequency
    sampling_intervals = np.diff(timestamps)
    mean_sampling_interval = np.mean(sampling_intervals)
    fs = 1.0 / mean_sampling_interval  # Sampling frequency in Hz
    print(f"Estimated Sampling Frequency: {fs:.2f} Hz")
    
    # Check for regular sampling
    if not np.allclose(sampling_intervals, mean_sampling_interval, atol=1e-4):
        print("Warning: Data is not sampled at regular intervals. Resampling data to regular intervals.")
        
        try:
            # Resample data using scipy.interpolate.interp1d
            regular_timestamps, resampled_data = resample_data(timestamps, data_to_analyze, fs)
            print(f"Data resampled to regular intervals at {fs:.2f} Hz.")
        except Exception as e:
            print(f"Error during resampling: {e}")
            exit(1)
        
        # Verify resampled data
        print(f"Resampled Data To Analyze - min: {resampled_data.min()}, max: {resampled_data.max()}, mean: {resampled_data.mean()}, std: {resampled_data.std()}")
        
        # Update data_to_analyze
        data_to_analyze = resampled_data
    else:
        print("Data is sampled at regular intervals.")
    
    # Verify the data to analyze
    print(f"Data To Analyze - min: {data_to_analyze.min()}, max: {data_to_analyze.max()}, mean: {data_to_analyze.mean()}, std: {data_to_analyze.std()}")
    
    # Proceed only if data has variability
    if data_to_analyze.std() == 0:
        print("Data to analyze has zero standard deviation. Cannot compute Allan Variance.")
        exit(1)
    
    # Compute Allan Variance using allantools without the 'method' keyword
    try:
        taus, avar, adev, _ = allantools.adev(
            data_to_analyze,
            rate=fs,
            data_type="freq",
            taus=None  # Let allantools decide the taus
        )
        
        # Debugging: Print statistics of taus and avar
        print(f"Number of taus: {len(taus)}")
        print(f"taus - min: {taus.min()}, max: {taus.max()}")
        print(f"avar - min: {avar.min()}, max: {avar.max()}")
        
        # Filter out non-positive avar values
        positive_indices = avar > 0
        taus_positive = taus[positive_indices]
        avar_positive = avar[positive_indices]
        
        if len(avar_positive) == 0:
            print("No positive Allan Variance values to plot.")
            exit(1)
        
    except Exception as e:
        print(f"Error computing Allan Variance: {e}")
        exit(1)
    
    # Plot Allan Deviation with filtered data
    plt.figure(figsize=(10, 6))
    plt.loglog(taus_positive, np.sqrt(avar_positive), label='Allan Deviation')
    plt.xlabel('Averaging Time (s)')
    plt.ylabel('Allan Deviation')
    plt.title('Allan Deviation for Gyroscope X-axis')
    plt.grid(True, which='both', ls='--')
    plt.legend()
    plt.show()
    
    # Optional: Compute Allan Variance for all axes
    axes = ['gyro_x', 'gyro_y', 'gyro_z']
    gyro_axes = [gyro_x, gyro_y, gyro_z]
    
    if not np.allclose(sampling_intervals, mean_sampling_interval, atol=1e-4):
        # If resampled, use the regularized data
        try:
            regular_timestamps, resampled_gyro_x = resample_data(timestamps, gyro_x, fs)
            _, resampled_gyro_y = resample_data(timestamps, gyro_y, fs)
            _, resampled_gyro_z = resample_data(timestamps, gyro_z, fs)
            gyro_axes = [resampled_gyro_x, resampled_gyro_y, resampled_gyro_z]
        except Exception as e:
            print(f"Error during resampling all axes: {e}")
            exit(1)
    
    plt.figure(figsize=(12, 8))
    
    for axis_name, data_axis in zip(axes, gyro_axes):
        # Check if data_axis has variability
        if data_axis.std() == 0:
            print(f"Data for {axis_name} has zero standard deviation. Skipping Allan Variance computation.")
            continue
        
        try:
            taus_axis, avar_axis, adev_axis, _ = allantools.adev(
                data_axis,
                rate=fs,
                data_type="freq",
                taus=None
            )
            
            # Filter non-positive avar values
            positive_indices_axis = avar_axis > 0
            taus_axis_positive = taus_axis[positive_indices_axis]
            avar_axis_positive = avar_axis[positive_indices_axis]
            
            if len(avar_axis_positive) == 0:
                print(f"No positive Allan Variance values to plot for {axis_name}.")
                continue
            
            plt.loglog(taus_axis_positive, np.sqrt(avar_axis_positive), label=f'Allan Deviation {axis_name}')
        
        except Exception as e:
            print(f"Error computing Allan Variance for {axis_name}: {e}")
    
    plt.xlabel('Averaging Time (s)')
    plt.ylabel('Allan Deviation')
    plt.title('Allan Deviation for All Gyroscope Axes')
    plt.grid(True, which='both', ls='--')
    plt.legend()
    plt.show()
