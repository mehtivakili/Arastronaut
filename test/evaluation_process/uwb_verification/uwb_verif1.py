import serial
import re
import matplotlib.pyplot as plt
from collections import deque
import time
import csv

# Serial port configuration
SERIAL_PORT = 'COM5'  # Replace with your actual serial port
BAUD_RATE = 115200
TIMEOUT = 0.01  # Set a small timeout for faster reading

# Initialize deque to store data for real-time plotting
time_data = deque(maxlen=1000)  # Store 10 times more time points (increase deque size)
distance_data = deque(maxlen=1000)

start_time = None  # Initialize start_time

# Create a CSV file to save time and distance
csv_file = open('time_distance_data.csv', mode='w', newline='')  # Open the file in write mode
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time (s)', 'Distance (m)'])  # Write the header

# Function to process a single packet of data
def process_packet(packet):
    global start_time
    try:
        # Regular expression to extract the elapsed time and distance
        match = re.search(r'Time elapsed: (\d+) ms,[^,]+, (\d+\.\d+)', packet)
        if match:
            elapsed_time = int(match.group(1)) / 1000  # Convert milliseconds to seconds
            distance = float(match.group(2))
            
            # Set the start_time when the first valid packet is processed
            if start_time is None:
                start_time = elapsed_time
            
            # Adjust elapsed time to start from zero
            adjusted_time = elapsed_time - start_time
            return adjusted_time, distance
    except ValueError as e:
        print(f"Error processing packet: {e}")
    return None, None

# Function to read data from serial port
def read_serial_data(serial_port):
    buffer = ''
    while True:
        try:
            byte_data = serial_port.read(1024)  # Read 1KB at a time from serial
            if byte_data:
                buffer += byte_data.decode('utf-8', errors='ignore')
                # Split the buffer using the separator 'bc'
                packets = buffer.split('bc')

                # Process all complete packets except the last (incomplete) one
                for packet in packets[:-1]:
                    yield packet.strip()
                # Keep the incomplete packet in the buffer
                buffer = packets[-1]
        except serial.SerialTimeoutException:
            pass  # Ignore timeout exceptions and continue reading

# Main function to initialize serial connection and handle real-time updates
def main():
    # Open the serial port
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
        # Create a new figure and axis for plotting
        fig, ax = plt.subplots()
        line, = ax.plot([], [], 'b-')  # Initial empty plot

        # Adjust the x-axis to handle more time data
        ax.set_xlim(0, 100)  # Set to 100 seconds to show 10 times more data
        ax.set_ylim(0, 10)   # Fixed Y-axis range (distance from 0 to 10 meters)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Distance (m)')
        ax.set_title('Real-time Distance Data')

        # Show the plot window before entering the loop
        plt.show(block=False)

        # Debug: print initial state
        print("Starting real-time plotting...")

        # Read and plot serial data in real-time
        for packet in read_serial_data(ser):
            elapsed_time, distance = process_packet(packet)
            if elapsed_time is not None and distance is not None:
                # Debug: show the parsed data
                print(f"Time: {elapsed_time:.3f} s, Distance: {distance:.2f} m")
                
                # Append the data to deque
                time_data.append(elapsed_time)
                distance_data.append(distance)

                # Save time and distance to CSV file
                csv_writer.writerow([elapsed_time, distance])  # Write to CSV

                # Update the plot data
                line.set_xdata(list(time_data))
                line.set_ydata(list(distance_data))

                # Adjust x-axis limits based on the current data range
                if len(time_data) > 0:
                    ax.set_xlim(min(time_data), min(time_data) + 100)  # Show a sliding window of 100 seconds

                ax.relim()  # Recalculate limits for the axis
                ax.autoscale_view()  # Rescale the view

                # Update the plot
                fig.canvas.draw()
                fig.canvas.flush_events()

                # Small delay to allow the plot to refresh
                time.sleep(0.05)

        # After the loop, keep the plot window open
        plt.show()

    # Close the CSV file after collecting data
    csv_file.close()

if __name__ == "__main__":
    main()
