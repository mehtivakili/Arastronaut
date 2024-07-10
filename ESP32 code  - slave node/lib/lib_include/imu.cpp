#include <Arduino.h> // Include Arduino base library
#include <SPI.h>     // Include SPI library
#include "imu.h"     // Include the IMU header file
#include "config.h"  // Include configuration file

SPIClass mySPI(HSPI);          // Create an SPIClass object
Bmi088Accel accel(SPI, 32);    // Create an accelerometer object
Bmi088Gyro gyro(SPI, 25);      // Create a gyroscope object

void initIMU() {
  int status;
  while (!Serial) {} // Wait for serial connection
  status = accel.begin(); // Initialize accelerometer
  if (status < 0) {
    Serial.println("Accel Initialization Error"); // Print error if initialization fails
    Serial.println(status); // Print status code
    while (1) {} // Halt execution
  }
  status = gyro.begin(); // Initialize gyroscope
  if (status < 0) {
    Serial.println("Gyro Initialization Error"); // Print error if initialization fails
    Serial.println(status); // Print status code
    while (1) {} // Halt execution
  }
}

void calibrateIMU(float raw_data[3], float calibrated_data[3], float bias[3], float K[3][3], float T[3][3]) {
  for (int i = 0; i < 3; i++) {
    calibrated_data[i] = 0;
    for (int j = 0; j < 3; j++) {
      calibrated_data[i] += (K[i][j] * T[j][j]) * (raw_data[j] - bias[j]);
    }
  }
}

void logIMUData(const char* label, float data[3]) {
  Serial.print(label); // Print label
  Serial.print(": ");
  Serial.print(data[0]); // Print X-axis data
  Serial.print("\t");
  Serial.print(data[1]); // Print Y-axis data
  Serial.print("\t");
  Serial.print(data[2]); // Print Z-axis data
  Serial.print("\n");
}

void readAndPrintIMUData() {
  accel.readSensor(); // Read accelerometer data
  gyro.readSensor(); // Read gyroscope data

  float acce_raw[3] = {accel.getAccelX_mss(), accel.getAccelY_mss(), accel.getAccelZ_mss()};
  float gyro_raw[3] = {gyro.getGyroX_rads(), gyro.getGyroY_rads(), gyro.getGyroZ_rads()};
  float acce_calibrated[3];
  float gyro_calibrated[3];

  if (IMU_CALIBRATED) {
    // Calibrate accelerometer and gyroscope data
    calibrateIMU(acce_raw, acce_calibrated, acce_bias, Ka, Ta);
    calibrateIMU(gyro_raw, gyro_calibrated, gyro_bias, Kg, Tg);

    // Log calibrated data
    logIMUData("Accel", acce_calibrated);
    logIMUData("Gyro", gyro_calibrated);
  } else {
    // Log raw data
    logIMUData("Accel", acce_raw);
    logIMUData("Gyro", gyro_raw);
  }

  delay(20); // Delay for 20 milliseconds
}
