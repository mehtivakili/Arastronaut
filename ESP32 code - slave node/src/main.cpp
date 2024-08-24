#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include "BMI088.h"
#include "DW1000.h"
#include "DW1000Ranging.h"
#include "esp_system.h"
#include "esp_mac.h"        // Include this as suggested
#include "esp_task_wdt.h"   // For Task Watchdog Timer functions
#include "esp_int_wdt.h"    // For Interrupt Watchdog Timer functions
#include "esp_timer.h"  // Include this for high-resolution timing
#include "QMC5883LCompass.h"

// Initialize the Task Watchdog Timer

Bmi088Accel accel(SPI, 32);
Bmi088Gyro gyro(SPI, 25);

QMC5883LCompass compass;


int16_t accelX_raw, accelY_raw, accelZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

double Tio = 0.0;

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 5;   // spi select pin

char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

const char* ap_ssid = "ESP32_AP";
const char* ap_password = "12345678";
const char* udpAddress = "192.168.4.100";  // The IP address of the Python client (update this to match your PC's IP)
const int udpPort = 12346;  // The port to send data to

WiFiUDP udp;
WebServer server(80);

bool sendData = true;
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
  IMU_UWB_DATA = 3,
  COMPASS = 4,
  IMU_COMPASS = 5,
  IMU_UWB_COMPASS = 6
};

Mode currentMode = IMU_ONLY;

const char* IMU_SEPARATOR = "abc/";
const char* UWB_SEPARATOR = "cba/";
const char* IMU_MAG_SEPARATOR = "img/";
const char* MAG_SEPARATOR = "mag/";

double myArray[7];
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

double myArray2[3];
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat2;

double myArray3[4];
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat3;

double myArray4[10];
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat4;

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
        calibration_in_progress = true;
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
      case COMPASS:
        currentMode = COMPASS;
        server.send(200, "text/plain", "Mode set to COMPASS");
        break;
      case IMU_COMPASS:
        currentMode = IMU_COMPASS;
        server.send(200, "text/plain", "Mode set to IMU_COMPASS");
        break;
      case IMU_UWB_COMPASS:
        currentMode = IMU_UWB_COMPASS;
        server.send(200, "text/plain", "Mode set to IMU_UWB_COMPASS");
        break;

        
      default:
        server.send(400, "text/plain", "Invalid mode");
        break;
    }
  } else {
    server.send(400, "text/plain", "Mode not specified");
  }
}

void safeRangingLoop() {
    unsigned long start = millis();
    while (true) {
        // Process a small part of the ranging loop
        DW1000Ranging.loop();

        // Break out of the loop if it has been running for too long
        if (millis() - start > 5) {  // Adjust the time as needed
            break;
        }

        // Reset watchdog to avoid triggering it
        esp_task_wdt_reset();
    }
}


bool sendUdpPacket(std::vector<uint8_t>& data, int maxRetries = 3) {
    int result = 0;
    for (int i = 0; i < maxRetries; ++i) {
        udp.beginPacket(udpAddress, udpPort);
        udp.write(data.data(), data.size());
        result = udp.endPacket();
        
        if (result == 1) {
            return true;  // Packet sent successfully
        }

        // If failed, wait for a short time before retrying
        delay(5);  // Adjust delay as needed
    }
    
    // Serial.print("Failed to send UDP packet after ");
    // Serial.print(maxRetries);
    // Serial.println(" retries.");
    return false;
}


void sendIMUData() {
  int64_t startTime = esp_timer_get_time();  // Start timing

  float tio_millis = static_cast<float>(millis());
  Tio = tio_millis / 1000.0;
  myArray[0] = Tio;
  myArray[1] = accelX_raw;
  myArray[2] = accelY_raw;
  myArray[3] = accelZ_raw;
  myArray[4] = gyroX_raw;
  myArray[5] = gyroY_raw;
  myArray[6] = gyroZ_raw;

  std::vector<uint8_t> imuData;

  // Add the IMU separator
  imuData.insert(imuData.end(), IMU_SEPARATOR, IMU_SEPARATOR + strlen(IMU_SEPARATOR));

  for (int i = 0; i < 7; i++) {
    binaryFloat hi;
    hi.floatingPoint = myArray[i];
    imuData.insert(imuData.end(), hi.binary, hi.binary + 4);
  }

  // Send the IMU data immediately
  // udp.beginPacket(udpAddress, udpPort);
  // udp.write(imuData.data(), imuData.size());
  // int result = udp.endPacket();

  // sendUdpPacket(imuData, 5);  // Send the IMU data with 3 retries

  // Send the data over serial
  Serial.write(imuData.data(), imuData.size());


  int64_t endTime = esp_timer_get_time();  // End timing
  // Serial.print("sendIMUData execution time: ");
  // Serial.print((endTime - startTime) / 1000.0);  // Convert microseconds to milliseconds
  // Serial.println(" ms");
  // if (result == 1) {
  //   Serial.println("IMU data sent successfully");
  // } else {
  //   Serial.print("Error sending IMU data: ");
  //   Serial.println(result);
  // }
}

void sendCompass() {
  compass.read();

  float tio_millis = static_cast<float>(millis());
  Tio = tio_millis / 1000.0;

  myArray3[0] = Tio;
  myArray3[1] = compass.getX();
  myArray3[2] = compass.getY();
  myArray3[3] = compass.getZ();

  // Serial.print("X: ");
  // Serial.print(myArray3[1]);
  // Serial.print(" Y: ");
  // Serial.print(myArray3[2]);
  // Serial.print(" Z: ");
  // Serial.print(myArray3[3]);
  // Serial.println();
  
  delay(25);
  
  std::vector<uint8_t> magData;
  // Add the MAG separator
  magData.insert(magData.end(), MAG_SEPARATOR, MAG_SEPARATOR + strlen(MAG_SEPARATOR));

  for (int i = 0; i < 4; i++) {
    binaryFloat hi;
    hi.floatingPoint = myArray3[i];
    magData.insert(magData.end(), hi.binary, hi.binary + 4);
  }

  currentBatchCount++;
    if (currentBatchCount >= batchSize) {

    // Send the IMU data immediately
    udp.beginPacket(udpAddress, udpPort);
    udp.write(magData.data(), magData.size());
    int result = udp.endPacket();
    if (result == 1) {
      // Serial.println("MAG data sent successfully");
    } else {
      // Serial.print("Error sending MAG data: ");
      // Serial.println(result);
    }
  }
}

void sendIMU_MAG() {

  int64_t startTime = esp_timer_get_time();  // Start timing

  float tio_millis = static_cast<float>(millis());
  Tio = tio_millis / 1000.0;
  myArray4[0] = Tio;
  myArray4[1] = accelX_raw;
  myArray4[2] = accelY_raw;
  myArray4[3] = accelZ_raw;
  myArray4[4] = gyroX_raw;
  myArray4[5] = gyroY_raw;
  myArray4[6] = gyroZ_raw;
  myArray4[7] = compass.getX();
  myArray4[8] = compass.getY();
  myArray4[9] = compass.getZ();

  std::vector<uint8_t> imuMag;

  // Add the IMU separator
  imuMag.insert(imuMag.end(), IMU_MAG_SEPARATOR, IMU_MAG_SEPARATOR + strlen(IMU_MAG_SEPARATOR));

  for (int i = 0; i < 10; i++) {
    binaryFloat hi;
    hi.floatingPoint = myArray4[i];
    imuMag.insert(imuMag.end(), hi.binary, hi.binary + 4);
  }

  sendUdpPacket(imuMag, 10);  // Send the IMU data with 3 retries


  // currentBatchCount++;
  //   if (currentBatchCount >= batchSize) {

    // Send the IMU data immediately
    // udp.beginPacket(udpAddress, udpPort);
    // udp.write(imuMag.data(), imuMag.size());
    // int result = udp.endPacket();

    
  //   if (result == 1) {
  //     // Serial.println("IMU data sent successfully");
  //   } else {
  //     Serial.print("Error sending IMU MAG data: ");
  //     Serial.println(result);
  //   }
  // }

  int64_t endTime = esp_timer_get_time();  // End timing
  // Serial.print("sendIMU_MagData execution time: ");
  // Serial.print((endTime - startTime) / 1000.0);  // Convert microseconds to milliseconds
  // Serial.println(" ms");
// }
}



void newDevice(DW1000Device *device) {
  // Serial.print("Device added: ");
  // Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  // Serial.print("delete inactive device: ");
  // Serial.println(device->getShortAddress(), HEX);
}

bool UWB_init = false;
void newRange() {
  int64_t startTime = esp_timer_get_time();  // Start timing

  std::vector<uint8_t> uwbData;
  float tio_millis = static_cast<float>(millis());
  Tio = tio_millis / 1000.0;
  myArray2[0] = Tio;

  float dist = DW1000Ranging.getDistantDevice()->getRange();
  // float address = 


  myArray2[1] = DW1000Ranging.getDistantDevice()->getShortAddress();
  myArray2[2] = dist;

  // Add the UWB separator
  uwbData.insert(uwbData.end(), UWB_SEPARATOR, UWB_SEPARATOR + strlen(UWB_SEPARATOR));

  for (int i = 0; i < 3; i++) {
    binaryFloat hi;
    hi.floatingPoint = myArray2[i];
    uwbData.insert(uwbData.end(), hi.binary, hi.binary + 4);
  }

  // Send the UWB data immediately
  // udp.beginPacket(udpAddress, udpPort);
  // udp.write(uwbData.data(), uwbData.size());
  // int result = udp.endPacket();

  sendUdpPacket(uwbData, 10);

  int64_t endTime = esp_timer_get_time();  // End timing
  // Serial.print("DW1000Ranging.loop execution time: ");
  // Serial.print((endTime - startTime) / 1000.0);  // Convert microseconds to milliseconds
  // Serial.println(" ms");
  // if (result == 1) {
  //   Serial.println("UWB data sent successfully");
  // } else {
  //   Serial.print("Error sending UWB data: ");
  //   Serial.println(result);
  // }
}



void setup() {
  Serial.begin(115200);

  if (accel.begin(Bmi088Accel::RANGE_12G, Bmi088Accel::ODR_200HZ_BW_80HZ) < 0 ||
      gyro.begin(Bmi088Gyro::RANGE_1000DPS, Bmi088Gyro::ODR_200HZ_BW_64HZ) < 0) {
    // Serial.println("Sensor initialization failed");
    while (1);
  }

  // Set up access point (AP) mode
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress AP_IP = WiFi.softAPIP();
  // Serial.print("AP SSID: ");
  // Serial.println(ap_ssid);
  // Serial.print("AP IP address: ");
  // Serial.println(AP_IP);

  compass.init();


  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/setBatch", handleSetBatch);
  server.on("/mode", handleModeChange);

  server.begin();


  // Serial.println("HTTP server started");
}

void logMemory() {
  static unsigned long lastLogTime = 0;
  unsigned long currentTime = millis();

  if (currentTime - lastLogTime >= 500) {  // Log memory every 500 milliseconds
    // Serial.print("Free heap: ");
    // Serial.println(ESP.getFreeHeap());
    lastLogTime = currentTime;
  }
}


void loop() {
  // server.handleClient();

    // esp_task_wdt_reset();  // Reset watchdog timer

  static unsigned long lastIMUSendTime = 0;
  static unsigned long lastRangeTime = 0;
  static unsigned long lastMAGSendTime = 0;
  static unsigned long lastIMUMAGSendTime = 0;
  static unsigned long last = 0;


  const int UWB_RATE = 15;    // at lower rate that 15Hz the output data will corrupt
  const int IMU_RATE = 5;
  const int MAG_RATE = 5;
  const int IMUMAG_RATE = 5;


  unsigned long currentTime = millis();

    if (sendData && currentTime - last >= 3000) {  // 10ms interval for web
      server.handleClient();
      last = currentTime;}

  switch (currentMode) {
    case IMU_ONLY:
      if (sendData && currentTime - lastIMUSendTime >= IMU_RATE) {  // 10ms interval for IMU data
        accel.readSensor();
        gyro.readSensor();
        accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
        gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);
        sendIMUData();
        lastIMUSendTime = currentTime;
      }
      break;

    case UWB_CALIBRATION:
      if (calibration_in_progress && currentTime - lastRangeTime >= UWB_RATE) {  // 50ms interval for UWB data
        DW1000Ranging.loop();
        lastRangeTime = currentTime;
      }
      break;

    case UWB_DATA:
      if (currentTime - lastRangeTime >= UWB_RATE) {
        DW1000Ranging.loop();
        lastRangeTime = currentTime;
      }
      break;

    case IMU_UWB_DATA:
      if (sendData && currentTime - lastIMUSendTime >= IMU_RATE) {  // IMU data every 10ms
        accel.readSensor();
        gyro.readSensor();
        accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
        gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);
        sendIMUData();
        lastIMUSendTime = currentTime;
      } 
      if (currentTime - lastRangeTime >= UWB_RATE) {  // UWB data every 50ms
        safeRangingLoop();  // Call the safe loop function
        lastRangeTime = currentTime;
      }
      break;

    case COMPASS:
      if (sendData && currentTime - lastMAGSendTime >= MAG_RATE) {  // IMU data every 10ms
        sendCompass();
        lastMAGSendTime = currentTime;
      }
      break;

    case IMU_COMPASS:
      if (sendData && currentTime - lastIMUMAGSendTime >= IMUMAG_RATE) {  // IMU data every 10ms

        accel.readSensor();
        gyro.readSensor();
        accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
        gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);
        sendIMU_MAG();
        lastIMUMAGSendTime = currentTime;
      }
      break;

    case IMU_UWB_COMPASS:
      if (sendData && currentTime - lastIMUMAGSendTime >= 10) {  // IMU data every 10ms
        compass.read();
        accel.readSensor();
        gyro.readSensor();
        accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
        gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);
        sendIMU_MAG();
        lastIMUMAGSendTime = currentTime;
      }
      if (currentTime - lastRangeTime >= 15) { 
        safeRangingLoop();  // Call the safe loop function
        lastRangeTime = currentTime;
      }        
      break;

    default:
      break;
  }
}
