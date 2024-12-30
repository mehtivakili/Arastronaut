import serial
import time
import csv

# Initialize serial communication with the ESP32
ser = serial.Serial('COM7', 115200)  # Replace with your serial port
ser.flushInput()

# Constants
PULSES_PER_REV = 600  # Encoder has 600 pulses per 360° rotation
DEGREES_PER_PULSE = 360.0 / PULSES_PER_REV  # Degrees per pulse

# Variables to store previous position and time for omega (angular velocity) calculation
prev_position = None
prev_time = None
prev_angle = None

# Set initial angle (hard-coded) and starting position
initial_angle_offset = 0 # Set initial position angle (e.g., starting at -150°)
invert_direction = True  # Set to True to invert encoder direction

# Flag to indicate first data point processing
first_data_point = True


def calculate_angle_omega(position, current_time):
    global prev_position, prev_time, prev_angle, first_data_point

    # Invert the encoder position if needed
    if invert_direction:
        position = -position

    # Calculate raw angle based on position
    raw_angle = (position % PULSES_PER_REV) * DEGREES_PER_PULSE

    # Handle the first data point separately
    if first_data_point:
        first_data_point = False
        prev_position = position
        prev_time = current_time
        prev_angle = initial_angle_offset
        return prev_angle, 0.0  # No angular velocity on the first data point

    # Calculate the angle with the initial offset
    angle = raw_angle + initial_angle_offset

    # Wrap the angle within -180° to 180°
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360

    # Calculate omega (angular velocity)
    if prev_position != position:
        delta_time = current_time - prev_time
        delta_angle = angle - prev_angle
        omega = delta_angle / delta_time if delta_time > 0 else 0
    else:
        omega = 0

    # Update previous values
    prev_position = position
    prev_time = current_time
    prev_angle = angle

    return angle, omega


def read_encoder_data():
    global prev_position, prev_time

    buffer = bytearray()

    # Open a CSV file to save the encoder data
    with open('dataset-encoder20.csv', 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        # Write the header
        csvwriter.writerow(['timestamp_ns', 'angle_deg'])

        try:
            while True:
                current_time = time.monotonic()  # More precise for timing

                # Read data if available in the serial buffer
                if ser.in_waiting > 0:
                    data = ser.read(min(ser.in_waiting, 64))  # Read available data
                    buffer.extend(data)

                    # Process lines from the buffer
                    while b'\n' in buffer:
                        line, buffer = buffer.split(b'\n', 1)
                        line = line.decode('utf-8').strip()

                        # Extract position data
                        try:
                            position = int(line.split(':')[-1].strip())
                        except (ValueError, IndexError):
                            continue  # Skip invalid data

                        # Calculate angle and omega
                        angle, omega = calculate_angle_omega(position, current_time)

                        # Get system timestamp in nanoseconds
                        system_timestamp_ns = int(time.time_ns())

                        # Save to CSV (only timestamp and angle)
                        csvwriter.writerow([system_timestamp_ns, angle])
                        csvfile.flush()  # Ensure data is written to the file immediately

                        # Print the position, angle, and omega
                        print(f"Position: {position}, Angle: {angle:.2f}°, Omega: {omega:.2f}°/s")

                # If no data, omega remains 0
                else:
                    time.sleep(0.01)  # Small sleep to avoid busy waiting

        except KeyboardInterrupt:
            ser.close()
            print("Program terminated")


if __name__ == "__main__":
    print("Starting to read encoder data and calculate angle/omega...")
    read_encoder_data()
