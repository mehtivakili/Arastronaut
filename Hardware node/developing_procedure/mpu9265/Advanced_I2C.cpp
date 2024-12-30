#include <MPU9250.h>  // Ensure you have the correct IMU library installed

MPU9250 IMU(Wire,0x68);

// Define the desired loop interval in microseconds (5 ms = 5000 µs)
const unsigned long LOOP_INTERVAL = 5000;

// Variable to store the last loop time
unsigned long lastLoopTime = 0;

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);
  
  // Initialize the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1); // Halt execution
  }

  // Configure IMU settings
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);          // +/-4G
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);       // +/-1000 deg/s
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ); // 10 Hz
  IMU.setSrd(4); // Sample Rate Divider: (1 + SRD) = 5 -> 200 Hz if internal rate is 1000 Hz

  // Optional: Verify IMU settings
  Serial.println("IMU Initialized with 200 Hz data rate.");
}

void loop() {
  unsigned long currentTime = micros();
  
  // Check if it's time to run the loop
  if (currentTime - lastLoopTime >= LOOP_INTERVAL * 1000) { // Convert ms to µs
    lastLoopTime += LOOP_INTERVAL * 1000; // Schedule next loop

    // Read sensor data from the IMU
    IMU.readSensor();

    // Get the current timestamp in milliseconds since the program started
    unsigned long timestamp = millis();

    // Prepare and send the sensor data over Serial, separated by tabs
    Serial.print(timestamp);
    Serial.print("\t");
    
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);
    Serial.print("\t");
    
    Serial.print(IMU.getMagX_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(), 6);
    Serial.print("\t");
    
    Serial.println(IMU.getTemperature_C(), 6);
    
    // Optional: Handle serial buffer overflow
    // If Serial is not ready, you might skip sending data or implement buffering
  }
  
  // Optional: Add other non-blocking tasks here
}
