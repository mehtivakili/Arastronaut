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

bool useCalibratedData = true;

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


int count_to_restart = 0;
void sendIMUData() {
    if (useCalibratedData){
    acce_calibrated[0] = ((int)(((Ka[0][0] * Ta[0][0]) + (Ka[0][1] * Ta[1][1]) + (Ka[0][2] * Ta[2][2])) * (accelX_raw - acce_bias[0]) * 1000)) / 1000.0;
    acce_calibrated[1] = ((int)(((Ka[1][1] * Ta[1][1]) + (Ka[1][2] * Ta[2][2])) * (accelY_raw - acce_bias[1]) * 1000)) / 1000.0;
    acce_calibrated[2] = ((int)(((Ka[2][2] * Ta[2][2])) * (accelZ_raw - acce_bias[2]) * 1000)) / 1000.0;

    gyro_calibrated[0] = ((int)(((Kg[0][0] * Tg[0][0]) + (Kg[0][1] * Tg[1][1]) + (Kg[0][2] * Tg[2][2])) * (gyroX_raw - gyro_bias[0]) * 1000)) / 1000.0;
    gyro_calibrated[1] = ((int)(((Kg[1][0] * Tg[1][0]) + (Kg[1][1] * Tg[1][1]) + (Kg[1][2] * Tg[2][2])) * (gyroY_raw - gyro_bias[1]) * 1000)) / 1000.0;
    gyro_calibrated[2] = ((int)(((Kg[2][0] * Tg[2][0]) + (Kg[2][1] * Tg[2][1]) + (Kg[2][2] * Tg[2][2])) * (gyroZ_raw - gyro_bias[2]) * 1000)) / 1000.0;
    
    // Serial.print(acce_calibrated[0]);

  }else{
    acce_calibrated[0] = accelX_raw;
    acce_calibrated[1] = accelY_raw;
    acce_calibrated[2] = accelZ_raw;
    gyro_calibrated[0] = gyroX_raw;
    gyro_calibrated[1] = gyroY_raw;
    gyro_calibrated[2] = gyroZ_raw;
  }
  float tio_millis =static_cast<float>(millis());
  Tio = tio_millis/1000.0;
  myArray[0] = Tio;
  myArray[1] = acce_calibrated[0];
  myArray[2] = acce_calibrated[1];
  myArray[3] = acce_calibrated[2];
  myArray[4] = gyro_calibrated[0];
  myArray[5] = gyro_calibrated[1];
  myArray[6] = gyro_calibrated[2];

   size_t size = sizeof(myArray);

  for(int i = 0; i < 7 ; i++){
    binaryFloat hi;
    hi.floatingPoint = myArray[i];
    Serial.write(hi.binary,4);
  }

    // uint8_t buffer[36]; // 8 bytes for double + 3*4 bytes for each float array (accel and gyro) + 4 bytes for CRC
  
  // Ensure data fits in the buffer
  static_assert(sizeof(Tio) == 8, "Size of double is not 8 bytes");
  static_assert(sizeof(acce_calibrated) == 12, "Size of accel data is not 12 bytes");
  static_assert(sizeof(gyro_calibrated) == 12, "Size of gyro data is not 12 bytes");

  // Pack data into buffer
  // memcpy(buffer, &Tio, sizeof(Tio));
  // memcpy(buffer + 8, acce_calibrated, sizeof(acce_calibrated));
  // memcpy(buffer + 20, gyro_calibrated, sizeof(gyro_calibrated));

  // // Decode the data for verification
  // double decodedTio;
  // float decodedAccel[3];
  // float decodedGyro[3];

  // memcpy(&decodedTio, buffer, sizeof(decodedTio));
  // memcpy(decodedAccel, buffer + 8, sizeof(decodedAccel));
  // memcpy(decodedGyro, buffer + 20, sizeof(decodedGyro));

    // Calculate CRC32 and append to the buffer
    // CRC32 crc;
    // crc.update(buffer, 32);
    // uint32_t crcValue = crc.finalize();
    // memcpy(buffer + 32, &crcValue, sizeof(crcValue));

    // // Send the buffer over serial
    // Serial.write(buffer, sizeof(buffer));

  // Send the data
  // bool result = webSocket.broadcastBIN(buffer, sizeof(buffer));
  // if (result) {
  //   // Serial.println("Data sent successfully");
  //   if(count_to_restart =! 0){
  //     count_to_restart -= 1;
  //   }
  // } else {
  //   Serial.println("Failed to send data");
  //   count_to_restart++;
  // }
  if (count_to_restart>15)
  {
    ESP.restart();
  }
  
}



void setup() {
  Serial.begin(115200);

  if (accel.begin(Bmi088Accel::RANGE_12G, Bmi088Accel::ODR_1600HZ_BW_234HZ) < 0 ||
      gyro.begin(Bmi088Gyro::RANGE_1000DPS, Bmi088Gyro::ODR_2000HZ_BW_230HZ) < 0) {
    Serial.println("Sensor initialization failed");
    while (1);
  }

  accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
  gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

}

void loop() {

  accel.readSensor();
  gyro.readSensor();

  accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
  gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);




  // Tio += 0.005;

  sendIMUData();
  delay(5);  // Adjust the delay to control the data sending rate
}
