import serial
import time

# Initialize serial communication with the ESP32
ser = serial.Serial('COM4', 915200)  # Replace with your serial port
ser.flushInput()

# Constants
PULSES_PER_REV = 583  # Encoder has 600 pulses per 360° rotation

# Variables to store previous position and time for omega (angular velocity) calculation
prev_position = None
prev_time = None
prev_angle = None

# Log file to save encoder data with angle and omega
log_file = "encoder_data.csv"

# Set initial angle (hard-coded) and starting position
initial_angle_offset = 168.0  # Set initial position angle (e.g., starting at -150°)
starting_position = 0  # Initial encoder position, change if necessary

# Invert direction of encoder (True = invert, False = no inversion)
invert_direction = True  # Set to True to invert encoder direction

# Flag to indicate whether the first data point has been processed
first_data_point = True  # Initially set to True


def calculate_angle_omega(position, current_time):
    global prev_position, prev_time, prev_angle, angle, first_data_point
    
    # Invert the encoder position if needed
    if invert_direction:
        position = -position

    # Calculate the raw angle of rotation in degrees based on the encoder position
    raw_angle = (position % PULSES_PER_REV) * (360.0 / PULSES_PER_REV)
    
    # Skip applying the initial angle offset for the first data point
    if not first_data_point:
        # Apply the initial offset (starting angle)
        angle = raw_angle + initial_angle_offset
    else:
        angle = raw_angle  # Do not apply the offset for the first reading
    
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

    # Once the first data point has been processed, set the flag to False
    if first_data_point:
        first_data_point = False

    return angle, omega




def read_encoder_data():
    global last_position, last_angle, last_omega
    
    # Initialize last known values to None
    last_position = 0  # Default starting position
    last_angle = 0.0   # Default starting angle
    last_omega = 0.0   # Default starting omega
    
    last_write_time = time.time()  # Initialize the time of the last write
    write_interval = 0.1  # Set the interval (in seconds) to write data to the file

    try:
        while True:
            current_time = time.time()

            # If no new data is received from the serial, we assume the device is stationary
            data_received = False  # Flag to track if data was received in this loop

            # Initialize the default values
            position = last_position
            angle = last_angle
            omega = last_omega

            if ser.in_waiting > 0:
                # Read a line from the serial buffer
                line = ser.readline().decode('utf-8').strip()
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
                print(f"Angle: {angle:.2f}°, Omega: {omega:.2f}°/s")

                # Store the current values for the next iteration
                last_position = position
                last_angle = angle
                last_omega = omega

            # If no new data is received, set omega to 0 and use the last known position/angle
            if not data_received:
                omega = 0
                # print(f"No data received. Setting omega to 0. Last position: {last_position}, Last angle: {last_angle}")

            # Always write the data to the CSV, even if no new data is received
            if current_time - last_write_time >= write_interval:
                with open(log_file, "a") as file:
                    file.write(f"{current_time},{last_angle:.2f}\n")

                # Print the data that was written
                print(f"Written to CSV - Timestamp: {current_time}, Position: {last_position}, Angle: {last_angle:.2f}, Omega: {omega:.2f}")
                
                last_write_time = current_time  # Reset the last write time

    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()    
        
if __name__ == "__main__":
    print("Starting to read encoder data and calculate angle/omega...")
    read_encoder_data()
