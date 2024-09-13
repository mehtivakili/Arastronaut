#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
unsigned long previousTime = 0; // Variable to store the previous timestamp
unsigned long currentTime = 0;  // Variable to store the current timestamp

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  // setting DLPF bandwidth to 92 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);
  // setting SRD to 4 for a 200 Hz update rate
  IMU.setSrd(4);

  // Set initial time
  previousTime = micros(); // Initialize the timestamp
}

void loop() {
  // Get the current time
  currentTime = micros(); // Get the current time in microseconds

  // Calculate the elapsed time in microseconds
  unsigned long elapsedTime = currentTime - previousTime;

  // Convert to seconds with millisecond precision
  float elapsedTimeSeconds = elapsedTime / 1000000.0; // Convert microseconds to seconds

  // Update the previous time
  previousTime = currentTime;

  // Read the sensor
  IMU.readSensor();

  // Display the data
  Serial.print(currentTime/1000000.0, 3);  // Print elapsed time in seconds with 3 decimal places (milliseconds precision)
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
  Serial.print(IMU.getTemperature_C(), 6);
  Serial.print("\t");

  // Print the elapsed time in seconds
  Serial.print("Elapsed Time (s): ");
  Serial.println(elapsedTimeSeconds, 3);  // Print elapsed time in seconds with milliseconds precision

  // Small delay
  delay(2);
}

