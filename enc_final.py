import serial
import time

# Initialize serial communication with the ESP32
ser = serial.Serial('COM7', 915200)  # Replace with your serial port
ser.flushInput()

# Constants
PULSES_PER_REV = 600  # Encoder has 600 pulses per 360° rotation

# Define buffer size
BUFFER_SIZE = 1024

# Variables to store previous position and time for omega (angular velocity) calculation
prev_position = None
prev_time = None
prev_angle = None

# Set initial angle (hard-coded) and starting position
initial_angle_offset = 143  # Set initial position angle (e.g., starting at -150°)
starting_position = 0  # Initial encoder position, change if necessary

# Invert direction of encoder (True = invert, False = no inversion)
invert_direction = True # Set to True to invert encoder direction

# Flag to indicate whether the first data point has been processed
first_data_point = True  # Initially set to True


def calculate_angle_omega(position, current_time):
    global prev_position, prev_time, prev_angle, angle, first_data_point, initial_angle_offset, starting_position
    
    # Invert the encoder position if needed
    if invert_direction:
        position = -position

    # Calculate the raw angle of rotation in degrees based on the encoder position
    raw_angle = (position % PULSES_PER_REV) * (360.0 / PULSES_PER_REV)
    
    # Handle the first data point separately to apply the offset correctly
    if first_data_point:
        # Set the initial position as zero and apply the initial offset
        starting_position = position  # Set the starting position at the first reading
        angle = initial_angle_offset  # Set the angle as the initial offset
        first_change = (1 / PULSES_PER_REV) * 360  # Calculate the first change
        angle += first_change  # Add the first change to the angle
        
        # Set the flag to indicate the first data point has been processed
        first_data_point = False
    else:
        # Apply the initial offset for subsequent data points
        angle = raw_angle + initial_angle_offset
    
    # Ensure the angle is wrapped within -180° to 180°
    if angle > 180:
        angle -= 360
    elif angle < -180:
        angle += 360

    # Initialize delta_angle and omega
    delta_angle = 0
    omega = 0

    # Calculate omega if position has changed
    if prev_position != position:
        # Calculate angular velocity (omega) in degrees per second
        if prev_time is not None:
            delta_time = current_time - prev_time
            # If there is a previous angle, calculate delta_angle
            if prev_angle is not None:
                delta_angle = angle - prev_angle
            # Only calculate omega if delta_time is not zero
            if delta_time > 0:
                omega = delta_angle / delta_time
            else:
                omega = 0  # If no time has passed, omega should be zero
    else:
        # If the position hasn't changed, omega should be 0
        omega = 0

    # Update previous position, time, and angle for the next calculation
    prev_position = position
    prev_time = current_time
    prev_angle = angle

    return angle, omega


def read_encoder_data():
    global last_position, last_angle, last_omega
    
    # Initialize last known values to None
    last_position = 0  # Default starting position
    last_angle = 0.0   # Default starting angle
    last_omega = 0.0   # Default starting omega
    
    # Initialize the serial buffer
    buffer = bytearray()

    try:
        while True:
            current_time = time.time()

            # If no new data is received from the serial, we assume the device is stationary
            data_received = False  # Flag to track if data was received in this loop

            # Initialize the default values
            position = last_position
            angle = last_angle
            omega = last_omega

            # Read data into the buffer if available
            if ser.in_waiting > 0:
                # Read data from serial and append to the buffer
                data = ser.read(min(ser.in_waiting, BUFFER_SIZE - len(buffer)))
                buffer.extend(data)
                
                # Process the buffer if it contains at least one line
                while b'\n' in buffer:
                    # Split the buffer at the first newline character
                    line, buffer = buffer.split(b'\n', 1)
                    line = line.decode('utf-8').strip()
                    current_time = time.time()

                    # Parse position from the serial data
                    try:
                        position = int(line.split(':')[-1].strip()) - starting_position
                        data_received = True  # Mark that data was received
                    except (ValueError, IndexError) as e:
                        print(f"Invalid data received: {line}")
                        continue

                    # Calculate angle and omega
                    angle, omega = calculate_angle_omega(position, current_time)

                    # Print the angle of rotation and angular velocity
                    print(f"Position: {position} Angle: {angle:.2f}°, Omega: {omega:.2f}°/s")

                    # Store the current values for the next iteration
                    last_position = position
                    last_angle = angle
                    last_omega = omega

            # If no new data is received, set omega to 0 and use the last known position/angle
            if not data_received:
                omega = 0

    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()

if __name__ == "__main__":
    print("Starting to read encoder data and calculate angle/omega...")
    read_encoder_data()
