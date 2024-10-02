#include <Wire.h>
#include <Arduino.h>

#define SDA_PIN 21  // Default SDA pin for ESP32
#define SCL_PIN 22  // Default SCL pin for ESP32
#define I2C_FREQ 400000 // 400kHz for faster communication
#define MPU9250_ADDRESS 0x68  // I2C address
#define PWR_MGMT_1      0x6B
#define CLOCK_SEL_PLL   0x01

bool writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(reg);
  Wire.write(data);
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    Serial.print("I2C Write Error: ");
    Serial.println(result);
    return false;
  }
  return true;
}


bool readRegister(uint8_t reg, uint8_t &data) {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) { // Restart for read
    Serial.println("I2C Read Transmission Error");
    return false;
  }
  
  Wire.requestFrom(MPU9250_ADDRESS, (uint8_t)1);
  if (Wire.available()) {
    data = Wire.read();
    return true;
  }
  return false;
}



void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);
  Serial.println("I2C Initialized");
  
  // Optionally, add a delay to allow MPU9250 to power up
  delay(100);
  
  // Attempt to write to PWR_MGMT_1
  Serial.println("Setting clock source to PLL...");
  if (writeRegister(PWR_MGMT_1, CLOCK_SEL_PLL)) {
    Serial.println("Clock source set to PLL successfully.");
  } else {
    Serial.println("Failed to set clock source to PLL.");
  }
  
  // Read back PWR_MGMT_1 to verify
  uint8_t pwr_mgmt_1;
  if (readRegister(PWR_MGMT_1, pwr_mgmt_1)) {
    Serial.print("PWR_MGMT_1 Register: 0x");
    Serial.println(pwr_mgmt_1, HEX);
  } else {
    Serial.println("Failed to read PWR_MGMT_1 register.");
  }
}

void loop() {
  // Your main code
}
