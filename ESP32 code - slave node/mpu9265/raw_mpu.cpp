#include "MPU9250.h"

// Custom class to extend the MPU9250 library and provide raw data getters
class MPU9250Extended : public MPU9250 {
public:
  MPU9250Extended(TwoWire &bus, uint8_t address) : MPU9250(bus, address) {}

  int16_t getAccelXRaw() { return _axcounts; }
  int16_t getAccelYRaw() { return _aycounts; }
  int16_t getAccelZRaw() { return _azcounts; }

  int16_t getGyroXRaw() { return _gxcounts; }
  int16_t getGyroYRaw() { return _gycounts; }
  int16_t getGyroZRaw() { return _gzcounts; }

  int16_t getMagXRaw() { return _hxcounts; }
  int16_t getMagYRaw() { return _hycounts; }
  int16_t getMagZRaw() { return _hzcounts; }

  int16_t getTemperatureRaw() { return _tcounts; }
};

// MPU9250 sensor object, using I2C bus with address 0x68
MPU9250Extended IMU(Wire, 0x68);

// Variables to store raw sensor data
int16_t accelXRaw, accelYRaw, accelZRaw;
int16_t gyroXRaw, gyroYRaw, gyroZRaw;
int16_t magXRaw, magYRaw, magZRaw;
int16_t tempRaw;
int status;

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  while (!Serial) {}
  status = IMU.begin();
  // Initialize the IMU
  if (status < 0) {
    Serial.print("IMU initialization failed. status = ");
    Serial.println(status);
    while (1);
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_92HZ);
  IMU.setSrd(9); // Set sample rate divider to 19 for 50 Hz output data rate
}

void loop() {
  // Read sensor data
  IMU.readSensor();

  // Extract raw data directly
  accelXRaw = IMU.getAccelXRaw();
  accelYRaw = IMU.getAccelYRaw();
  accelZRaw = IMU.getAccelZRaw();
  
  gyroXRaw = IMU.getGyroXRaw();
  gyroYRaw = IMU.getGyroYRaw();
  gyroZRaw = IMU.getGyroZRaw();

  magXRaw = IMU.getMagXRaw();
  magYRaw = IMU.getMagYRaw();
  magZRaw = IMU.getMagZRaw();

  tempRaw = IMU.getTemperatureRaw();

  // Print raw data to Serial
  Serial.print("Accel Raw: ");
  Serial.print(accelXRaw);
  Serial.print(", ");
  Serial.print(accelYRaw);
  Serial.print(", ");
  Serial.print(accelZRaw);
  Serial.print("\t");

  Serial.print("Gyro Raw: ");
  Serial.print(gyroXRaw);
  Serial.print(", ");
  Serial.print(gyroYRaw);
  Serial.print(", ");
  Serial.print(gyroZRaw);
  Serial.print("\t");

  Serial.print("Mag Raw: ");
  Serial.print(magXRaw);
  Serial.print(", ");
  Serial.print(magYRaw);
  Serial.print(", ");
  Serial.print(magZRaw);
  Serial.print("\t");

  Serial.print("Temp Raw: ");
  Serial.println(tempRaw);

  delay(50);  // Wait for 1 second before repeating
}
