#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include "BMI088.h"

Bmi088Accel accel(SPI, 32);
Bmi088Gyro gyro(SPI, 25);

int16_t accelX_raw, accelY_raw, accelZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

double Tio = 0.0;

const char* ssid = "ESP32_AP";
const char* password = "123456789";
const char* udpAddress = "192.168.4.2";  // The IP address of the Python client
const int udpPort = 12345;  // The port to send data to

WiFiUDP udp;
WebServer server(80);

bool sendData = false;

double myArray[7];
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

const char* check = "abc/";

void handleStart() {
  sendData = true;
  server.send(200, "text/plain", "Data transmission started");
}

void handleStop() {
  sendData = false;
  server.send(200, "text/plain", "Data transmission stopped");
}

void sendIMUData() {
  float tio_millis = static_cast<float>(millis());
  Tio = tio_millis / 1000.0;
  myArray[0] = Tio;
  myArray[1] = accelX_raw;
  myArray[2] = accelY_raw;
  myArray[3] = accelZ_raw;
  myArray[4] = gyroX_raw;
  myArray[5] = gyroY_raw;
  myArray[6] = gyroZ_raw;

  udp.beginPacket(udpAddress, udpPort);

  udp.write((const uint8_t*)check, strlen(check));  // Write the check variable

  for (int i = 0; i < 7; i++) {
    binaryFloat hi;
    hi.floatingPoint = myArray[i];
    udp.write(hi.binary, 4);
  }

  int result = udp.endPacket();
  if (result == 1) {
    Serial.println("Packet sent successfully");
  } else {
    Serial.print("Error sending packet: ");
    Serial.println(result);
  }
}

void setup() {
  Serial.begin(115200);

  if (accel.begin(Bmi088Accel::RANGE_12G, Bmi088Accel::ODR_200HZ_BW_80HZ) < 0 ||
      gyro.begin(Bmi088Gyro::RANGE_1000DPS, Bmi088Gyro::ODR_200HZ_BW_64HZ) < 0) {
    Serial.println("Sensor initialization failed");
    while (1);
  }

  WiFi.softAP(ssid, password, 1, 0);

  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.begin();

  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  if (sendData) {
    accel.readSensor();
    gyro.readSensor();

    accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
    gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

    sendIMUData();
    delay(5);
  }
}

