import serial
import time

# Initialize serial communication with the ESP32
ser = serial.Serial('COM4', 915200)  # Replace with your serial port
ser.flushInput()

# Constants
PULSES_PER_REV = 600  # Encoder has 600 pulses per 360° rotation

# Variables to store previous position and time for omega (angular velocity) calculation
prev_position = None
prev_time = None
prev_angle = None

# Log file to save encoder data with angle and omega
log_file = "encoder_angle_omega_data.txt"

# Set initial angle (hard-coded) and starting position
initial_angle_offset = 18.0  # Set initial position angle (e.g., starting at -150°)
starting_position = 0  # Initial encoder position, change if necessary

# Invert direction of encoder (True = invert, False = no inversion)
invert_direction = True  # Set to True to invert encoder direction

def calculate_angle_omega(position, current_time):
    global prev_position, prev_time, prev_angle
    
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

    # Calculate angular velocity (omega) in degrees per second
    if prev_time is not None:
        delta_time = current_time - prev_time
        delta_angle = angle - prev_angle if prev_angle is not None else 0
        if delta_time != 0:
            omega = delta_angle / delta_time
        else:
            omega = 0
    else:
        omega = 0

    # Update previous position, time, and angle for the next calculation
    prev_position = position
    prev_time = current_time
    prev_angle = angle

    return angle, omega

def read_encoder_data():
    try:
        while True:
            if ser.in_waiting > 0:
                # Read a line from the serial buffer
                line = ser.readline().decode('utf-8').strip()
                current_time = time.time()
                
                # Convert the position to an integer
                position = int(line.split(':')[-1].strip()) - starting_position

                # Calculate angle and omega
                angle, omega = calculate_angle_omega(position, current_time)

                # Print the angle of rotation and angular velocity
                print(f"Angle: {angle:.2f}°, Omega: {omega:.2f}°/s")

                # Optionally, save the data to a file
                with open(log_file, "a") as file:
                    file.write(f"{position},{angle:.2f},{omega:.2f}\n")

                # Small delay to avoid overwhelming the Serial Monitor
                # time.sleep(0.1)

    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()

if __name__ == "__main__":
    print("Starting to read encoder data and calculate angle/omega...")
    read_encoder_data()
