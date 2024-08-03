#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include "BMI088.h"
#include "DW1000.h"
#include "DW1000Ranging.h"

Bmi088Accel accel(SPI, 32);
Bmi088Gyro gyro(SPI, 25);

int16_t accelX_raw, accelY_raw, accelZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

double Tio = 0.0;

// Replace with your network credentials
const char* sta_ssid = "D-Link";
const char* sta_password = "09124151339";

const char* ap_ssid = "ESP32_AP";
const char* ap_password = "123456789";
const char* udpAddress = "192.168.4.2";  // The IP address of the Python client (update this to match your PC's IP)
const int udpPort = 12346;  // The port to send data to

WiFiUDP udp;
WebServer server(80);

bool sendData = false;
int batchSize = 1;  // Default batch size
int currentBatchCount = 0;

bool calibration_in_progress = false;

bool imuDataReady = false;
bool uwbDataReady = false;
std::vector<uint8_t> combinedData;


enum Mode {
  IMU_ONLY = 0,
  UWB_CALIBRATION = 1,
  UWB_DATA = 2,
  IMU_UWB_DATA = 3
};

Mode currentMode = IMU_ONLY;

const char* IMU_SEPARATOR = "abc/";
const char* UWB_SEPARATOR = "cba/";


double myArray[7];
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

const char* check = "abc/";
std::vector<uint8_t> batchData;

void handleStart() {
  sendData = true;
  server.send(200, "text/plain", "Data transmission started");
}

void handleStop() {
  sendData = false;
  server.send(200, "text/plain", "Data transmission stopped");
}

void handleSetBatch() {
  if (server.hasArg("batch")) {
    batchSize = server.arg("batch").toInt();
    server.send(200, "text/plain", "Batch size set to " + String(batchSize));
  } else {
    server.send(400, "text/plain", "Batch size not specified");
  }
}

void handleModeChange() {
  if (server.hasArg("mode")) {
    int mode = server.arg("mode").toInt();
    switch (mode) {
      case IMU_ONLY:
        currentMode = IMU_ONLY;
        server.send(200, "text/plain", "Mode set to IMU_ONLY");
        break;
      case UWB_CALIBRATION:
        currentMode = UWB_CALIBRATION;
        server.send(200, "text/plain", "Mode set to UWB_CALIBRATION");
        break;
      case UWB_DATA:
        currentMode = UWB_DATA;
        server.send(200, "text/plain", "Mode set to UWB_DATA");
        break;
      case IMU_UWB_DATA:
        currentMode = IMU_UWB_DATA;
        server.send(200, "text/plain", "Mode set to IMU_UWB_DATA");
        break;
      default:
        server.send(400, "text/plain", "Invalid mode");
        break;
    }
  } else {
    server.send(400, "text/plain", "Mode not specified");
  }
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

  // Append check string to batch data
  // Clear the batch data and add the IMU header
  combinedData.clear();
  combinedData.insert(combinedData.end(), IMU_SEPARATOR, IMU_SEPARATOR + strlen(IMU_SEPARATOR));

  for (int i = 0; i < 7; i++) {
    binaryFloat hi;
    hi.floatingPoint = myArray[i];
    // batchData.insert(batchData.end(), hi.binary, hi.binary + 4);
    combinedData.insert(combinedData.end(), hi.binary, hi.binary + 4);
  
  }
  // Send the IMU data immediately
  udp.beginPacket(udpAddress, udpPort);
  udp.write(batchData.data(), batchData.size());
  int result = udp.endPacket();
  if (result == 1) {
    Serial.println("IMU data sent successfully");
  } else {
    Serial.print("Error sending IMU data: ");
    Serial.println(result);
  }

  // currentBatchCount++;

  // if (currentBatchCount >= batchSize) {
  //   udp.beginPacket(udpAddress, udpPort);
  //   udp.write(batchData.data(), batchData.size());
  //   int result = udp.endPacket();
  //   if (result == 1) {
  //     Serial.println("Batch sent successfully");
  //   } else {
  //     Serial.print("Error sending batch: ");
  //     Serial.println(result);
  //   }

  //   batchData.clear();  // Clear the batch data
  //   currentBatchCount = 0;  // Reset the batch count
  // }
}
void sendCombinedData() {
  if (imuDataReady && uwbDataReady) {
    udp.beginPacket(udpAddress, udpPort);
    udp.write(combinedData.data(), combinedData.size());
    int result = udp.endPacket();
    if (result == 1) {
      Serial.println("Combined data sent successfully");
    } else {
      Serial.print("Error sending combined data: ");
      Serial.println(result);
    }

    combinedData.clear();
    imuDataReady = false;
    uwbDataReady = false;
  }
}
void newRange() {
  float dist = DW1000Ranging.getDistantDevice()->getRange();
  // Clear the batch data and add the UWB separator
  combinedData.clear();
  combinedData.insert(combinedData.end(), UWB_SEPARATOR, UWB_SEPARATOR + strlen(UWB_SEPARATOR));

  binaryFloat hi;
  hi.floatingPoint = dist;
  combinedData.insert(combinedData.end(), hi.binary, hi.binary + 4);

  // Send the UWB data immediately
  udp.beginPacket(udpAddress, udpPort);
  udp.write(combinedData.data(), combinedData.size());
  int result = udp.endPacket();
  if (result == 1) {
    Serial.println("UWB data sent successfully");
  } else {
    Serial.print("Error sending UWB data: ");
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

  // Connect to Wi-Fi network in station mode (STA)
  WiFi.begin(sta_ssid, sta_password);
  Serial.print("Connecting to WiFi: ");
  Serial.println(sta_ssid);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up access point (AP) mode
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress AP_IP = WiFi.softAPIP();
  Serial.print("AP SSID: ");
  Serial.println(ap_ssid);
  Serial.print("AP IP address: ");
  Serial.println(AP_IP);

  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/setBatch", handleSetBatch);
  server.on("/mode", handleModeChange);

  server.begin();

  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();

  switch (currentMode) {
    case IMU_ONLY:
      if (sendData) {
        accel.readSensor();
        gyro.readSensor();
        accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
        gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);
        sendIMUData();
        delay(5);
      }
      break;

    case UWB_CALIBRATION:
      if (calibration_in_progress) {
        DW1000Ranging.loop();
      }
      break;

    case UWB_DATA:
      DW1000Ranging.loop();
      break;

    case IMU_UWB_DATA:
      if (sendData) {
        accel.readSensor();
        gyro.readSensor();
        accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
        gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);
        sendIMUData();
        delay(5);
      }
      DW1000Ranging.loop();
      break;

    default:
      break;
  }
}

