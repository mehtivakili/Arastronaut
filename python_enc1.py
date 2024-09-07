import serial
import time

# Initialize serial communication with the ESP32
ser = serial.Serial('COM7', 915200)  # Replace with your serial port
ser.flushInput()

# Log file to save encoder data
log_file = "encoder_data.txt"

def read_encoder_data():
    try:
        while True:
            if ser.in_waiting > 0:
                # Read a line from the serial buffer
                line = ser.readline().decode('utf-8').strip()

                # Print the encoder position
                print(f"Encoder Position: {line}")

                # Optionally, save the data to a file
                with open(log_file, "a") as file:
                    file.write(f"{line}\n")

                # Small delay to avoid overwhelming the Serial Monitor
                # time.sleep(0.1)

    except KeyboardInterrupt:
        print("Exiting program.")
        ser.close()

if __name__ == "__main__":
    print("Starting to read encoder data from serial port...")
    read_encoder_data()
