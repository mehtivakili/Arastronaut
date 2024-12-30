import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import allantools

# Load data
file_path = 'gyro-202409301452.csv'  # Replace with your actual file path
data = pd.read_csv(file_path, sep='\s+', header=None, 
                   names=['timestamp', 'gyro_x', 'gyro_y', 'gyro_z'])

# Extract gyro data and timestamps
timestamps = data['timestamp'].values
gyro_x = data['gyro_x'].values
gyro_y = data['gyro_y'].values
gyro_z = data['gyro_z'].values

# Calculate the total duration and sampling interval
sampling_interval = np.mean(np.diff(timestamps))  # Mean sampling interval
max_duration = timestamps[-1] - timestamps[0]     # Total duration of your dataset

# Define the sampling frequency
Fs = 1.0 / sampling_interval

# Define tau values ranging from the sampling interval to the total duration
taus_manual = np.logspace(np.log10(sampling_interval), np.log10(max_duration), num=200)

# Calculate Allan deviation for each axis using manually defined taus
result_x = allantools.adev(gyro_x, rate=Fs, data_type="freq", taus=taus_manual)
taus_x, adev_x = result_x[0], result_x[1]

result_y = allantools.adev(gyro_y, rate=Fs, data_type="freq", taus=taus_manual)
taus_y, adev_y = result_y[0], result_y[1]

result_z = allantools.adev(gyro_z, rate=Fs, data_type="freq", taus=taus_manual)
taus_z, adev_z = result_z[0], result_z[1]

# Plot the Allan deviation
plt.figure(figsize=(10, 6))
plt.loglog(taus_x, adev_x, label='Gyro X')
plt.loglog(taus_y, adev_y, label='Gyro Y')
plt.loglog(taus_z, adev_z, label='Gyro Z')
plt.xlabel('Averaging Time (s)')
plt.ylabel('Allan Deviation (rad/s)')
plt.title('Allan Deviation of Gyroscope Data')
plt.legend()
plt.grid(True, which="both", ls="--")
plt.show()

