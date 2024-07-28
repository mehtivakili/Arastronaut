#include <WiFiManager.h>
#include <NetworkConsole.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "BMI088.h"
#include "config2.h"
#include <SPIFFS.h>
#include <CRC32.h>
#include <stdio.h>

Bmi088Accel accel(SPI, 32);
Bmi088Gyro gyro(SPI, 25);

bool useCalibratedData = false;

int16_t accelX_raw, accelY_raw, accelZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

float acce_calibrated[3];
float gyro_calibrated[3];

double Tio = 0.0;

float Ta[3][3] = {{1, -0.00546066, 0.00101399}, {0, 1, 0.00141895}, {0, 0, 1}};
float Ka[3][3] = {{0.00358347, 0, 0}, {0, 0.00358133, 0}, {0, 0, 0.00359205}};
float Tg[3][3] = {{1, -0.00614889, -0.000546488}, {0.0102258, 1, 0.000838491}, {0.00412113, 0.0020154, 1}};
float Kg[3][3] = {{0.000531972, 0, 0}, {0, 0.000531541, 0}, {0, 0, 0.000531}};
float acce_bias[3] = {-8.28051, -4.6756, -0.870355};
float gyro_bias[3] = {4.53855, 4.001, -1.9779};

String formatFloat(float value) {
  return String(value, 3);
}

double myArray[7];
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;


void sendIMUData() {
  
  float tio_millis =static_cast<float>(millis());
  Tio = tio_millis/1000.0;
  myArray[0] = Tio;
  myArray[1] = accelX_raw;
  myArray[2] = accelY_raw;
  myArray[3] = accelZ_raw;
  myArray[4] = gyroX_raw;
  myArray[5] = gyroY_raw;
  myArray[6] = gyroZ_raw;

   size_t size = sizeof(myArray);

  const char check = 'c';
  Serial.write(check);

  for(int i = 0; i < 7 ; i++){
    binaryFloat hi;
    hi.floatingPoint = myArray[i];
    Serial.write(hi.binary,4);
  }
  
}



void setup() {
  Serial.begin(115200);

  if (accel.begin(Bmi088Accel::RANGE_12G, Bmi088Accel::ODR_200HZ_BW_80HZ) < 0 ||
      gyro.begin(Bmi088Gyro::RANGE_1000DPS, Bmi088Gyro::ODR_200HZ_BW_64HZ) < 0) {
    Serial.println("Sensor initialization failed");
    while (1);
  }
}

void loop() {

  accel.readSensor();    // if you delete these two lines the data will be corrupted
  gyro.readSensor();

  accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
  gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

  sendIMUData();
  delay(5);  // Adjust the delay to control the data sending rate
}
