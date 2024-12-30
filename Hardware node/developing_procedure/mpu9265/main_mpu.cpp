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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

// Task handle
TaskHandle_t sendIMUTaskHandle = NULL;
volatile bool imuDataReadyToSend = false;

// Task handle
TaskHandle_t sendIMUMAGTaskHandle = NULL;
volatile bool imuMagDataReadyToSend = false;

bool rawData = true;

extern volatile int counter;
extern volatile int uwbReady;


// Initialize the Task Watchdog Timer

// Bmi088Accel accel(SPI, 32);
// Bmi088Gyro gyro(SPI, 25);
Bmi088 bmi(SPI, 32, 25);


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
const char* ap_password = "123456789";
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
  IMU_UWB_COMPASS = 6,
  IMU_ONLY_2 = 7,
  Terminate = 99
};

Mode currentMode = IMU_COMPASS;

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

void handleRawData(){
  if (rawData == true){
    rawData = false;
    server.send(200, "text/plain", "Raw data set to " + String(rawData));

  } else {
    rawData = true;
    server.send(200, "text/plain", "Raw data set to " + String(rawData));

  }
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
      case Terminate:
        currentMode = Terminate;
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

volatile int counter2 = 0;  // Use volatile to tell the compiler this variable can be changed unexpectedly
volatile int imuReady = 0;

unsigned long startTime = 0;
int Count = 0;
void imuInt(){
  // if (counter == 0){
      counter2 = counter2 + 1;

  // unsigned long startTime = millis();
  // }
  // unsigned long nowTime = millis();
  // if (nowTime - startTime> 1000){
  //   Count = counter;
  //   // counter = 0;
  // }
  imuReady = 1;
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

int callCounter = 0; // Counter to track the number of function calls
std::vector<uint8_t> bufferedImuData; // Buffer to store the first set of data

void sendIMUData() {
      // Increment the function call counter
    callCounter++;
  // int64_t startTime = esp_timer_get_time();  // Start timing in microseconds

    // Get the elapsed time in nanoseconds
  int64_t elapsedTimeNs = esp_timer_get_time() * 1000;  // Convert microseconds to nanoseconds

  // int64_t startTime = esp_timer_get_time();  // Start timing

  // float tio_millis = static_cast<float>(millis());
  // Tio = tio_millis / 1000.0;
  // myArray[0] = Tio;
  // if(rawData == true){
    myArray[1] = accelX_raw;
    myArray[2] = accelY_raw;
    myArray[3] = accelZ_raw;
    myArray[4] = gyroX_raw;
    myArray[5] = gyroY_raw;
    myArray[6] = gyroZ_raw;
  // } else {
  //   myArray[1] = bmi.getAccelX_mss();
  //   myArray[2] = bmi.getAccelY_mss();
  //   myArray[3] = bmi.getAccelZ_mss();
  //   myArray[4] = bmi.getGyroX_rads();
  //   myArray[5] = bmi.getGyroY_rads();
  //   myArray[6] = bmi.getGyroZ_rads();
  // }




  std::vector<uint8_t> imuData;

  // Add the IMU separator
  imuData.insert(imuData.end(), IMU_SEPARATOR, IMU_SEPARATOR + strlen(IMU_SEPARATOR));

    // Add the 64-bit timestamp to the data buffer
    imuData.insert(imuData.end(), reinterpret_cast<uint8_t*>(&elapsedTimeNs), reinterpret_cast<uint8_t*>(&elapsedTimeNs) + sizeof(int64_t));

  for (int i = 1; i < 7; i++) {
    binaryFloat hi;
    hi.floatingPoint = myArray[i];
    imuData.insert(imuData.end(), hi.binary, hi.binary + 4);
  }

    // Check if this is the first or second call
    if (callCounter == 1) {
        // Store the data in the buffer
        bufferedImuData = imuData;
    } else if (callCounter == 2) {
        // Send the buffered data (from the first call) and the current data (from the second call)
        udp.beginPacket(udpAddress, udpPort);

        // Send the buffered data first
        udp.write(bufferedImuData.data(), bufferedImuData.size());

        // Send the current data
        udp.write(imuData.data(), imuData.size());

        int result = udp.endPacket();

        // Reset the counter and clear the buffer after sending
        callCounter = 0;
        bufferedImuData.clear();
    }
  // sendUdpPacket(imuData, 5);  // Send the IMU data with 3 retries

  // Send the data over serial
  // Serial.write(imuData.data(), imuData.size());


  // int64_t endTime = esp_timer_get_time();  // End timing

  // int64_t endTime = esp_timer_get_time();  // End timing


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

void sendIMUTask(void * parameter) {
    for (;;) {
        if (imuDataReadyToSend) {
            imuDataReadyToSend = false;

            // bmi.readSensor();
            bmi.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw, &gyroX_raw, &gyroY_raw, &gyroZ_raw);


            // Send IMU data
            sendIMUData();

            // Optionally, add some delay or task yield if necessary
            vTaskDelay(1);
        }
    }
}



void sendCompass() {
  compass.read();
  std::vector<uint8_t> bufferedMagData; // Buffer to store the first set of data
  int64_t elapsedTimeNs = esp_timer_get_time() * 1000;  // Convert microseconds to nanoseconds
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
  
  delay(5);
  
  std::vector<uint8_t> magData;
  // Add the MAG separator
  magData.insert(magData.end(), MAG_SEPARATOR, MAG_SEPARATOR + strlen(MAG_SEPARATOR));

    // Add the 64-bit timestamp to the data buffer
    magData.insert(magData.end(), reinterpret_cast<uint8_t*>(&elapsedTimeNs), reinterpret_cast<uint8_t*>(&elapsedTimeNs) + sizeof(int64_t));

  for (int i = 1; i < 4; i++) {
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
// int callCounter = 0; // Counter to track the number of function calls
std::vector<uint8_t> bufferedImuMagData; // Buffer to store the first set of data

void sendIMU_MAG() {
    callCounter++;
        IMU.readSensor();

    // compass.read();

  // int64_t startTime = esp_timer_get_time();  // Start timing

  // float tio_millis = static_cast<float>(millis());
  // Tio = tio_millis / 1000.0;
      // Get the elapsed time in nanoseconds
  int64_t elapsedTimeNs = esp_timer_get_time() * 1000;  // Convert microseconds to nanoseconds
 
  // myArray4[0] = Tio;
  myArray4[1] = accelXRaw;
  myArray4[2] = accelYRaw;
  myArray4[3] = accelZRaw;
  myArray4[4] = gyroXRaw;
  myArray4[5] = gyroYRaw;
  myArray4[6] = gyroZRaw;
  myArray4[7] = magXRaw;
  myArray4[8] = magYRaw;
  myArray4[9] = magZRaw;

  // myArray4[1] = 0;
  // myArray4[2] = 0;
  // myArray4[3] = 0;
  // myArray4[4] = 0;
  // myArray4[5] = 0;
  // myArray4[6] = 0;
  // myArray4[7] = 0;
  // myArray4[8] = 0;
  // myArray4[9] = 0;

  // myArray4[1] = IMU.getAccelX_mss();
  // myArray4[2] = IMU.getAccelY_mss();
  // myArray4[3] = IMU.getAccelZ_mss();
  // myArray4[4] = IMU.getGyroX_rads();
  // myArray4[5] = IMU.getGyroY_rads();
  // myArray4[6] = IMU.getGyroZ_rads();
  // myArray4[7] = IMU.getMagX_uT();
  // myArray4[8] = IMU.getMagY_uT();
  // myArray4[9] = IMU.getMagZ_uT();

  for(int i = 1; i<4; i++){
    Serial.print(myArray4[i]);
    Serial.print(", ");
  }  
  Serial.println("");
  // myArray4[7] = compass.getX();
  // myArray4[8] = compass.getY();
  // myArray4[9] = compass.getZ();

  // Serial.print(compass.getX());
  // Serial.print(", ");
  // Serial.print(compass.getY());
  // Serial.print(", ");
  // Serial.println(compass.getZ());


  std::vector<uint8_t> imuMag;

  // Add the IMU separator
  imuMag.insert(imuMag.end(), IMU_MAG_SEPARATOR, IMU_MAG_SEPARATOR + strlen(IMU_MAG_SEPARATOR));

      // Add the 64-bit timestamp to the data buffer
    imuMag.insert(imuMag.end(), reinterpret_cast<uint8_t*>(&elapsedTimeNs), reinterpret_cast<uint8_t*>(&elapsedTimeNs) + sizeof(int64_t));

  for (int i = 1; i < 10; i++) {
    binaryFloat hi;
    hi.floatingPoint = myArray4[i];
    imuMag.insert(imuMag.end(), hi.binary, hi.binary + 4);
  }

      // Check if this is the first or second call
    if (callCounter == 1) {
        // Store the data in the buffer
        bufferedImuMagData = imuMag;
    } else if (callCounter == 2) {
        // Send the buffered data (from the first call) and the current data (from the second call)
        udp.beginPacket(udpAddress, udpPort);

        // Send the buffered data first
        udp.write(bufferedImuMagData.data(), bufferedImuMagData.size());

        // Send the current data
        udp.write(imuMag.data(), imuMag.size());

        int result = udp.endPacket();

        // Reset the counter and clear the buffer after sending
        callCounter = 0;
        bufferedImuMagData.clear();
    }

  // sendUdpPacket(imuMag, 10);  // Send the IMU data with 3 retries


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

  // int64_t endTime = esp_timer_get_time();  // End timing
  // Serial.print("sendIMU_MagData execution time: ");
  // Serial.print((endTime - startTime) / 1000.0);  // Convert microseconds to milliseconds
  // Serial.println(" ms");
// }
}

void sendIMUMAGTask(void * parameter) {
    for (;;) {
        if (imuMagDataReadyToSend) {
            imuMagDataReadyToSend = false;

            // bmi.readSensor();
            bmi.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw, &gyroX_raw, &gyroY_raw, &gyroZ_raw);

            // compass.read();
            // Serial.print(compass.getX());
            // Serial.print(", ");
            // Serial.print(compass.getY());
            // Serial.print(", ");
            // Serial.println(compass.getZ());
            // Send IMU data
            sendIMU_MAG();

            // Optionally, add some delay or task yield if necessary
            vTaskDelay(5);
        }
    }
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
  udp.beginPacket(udpAddress, udpPort);
  udp.write(uwbData.data(), uwbData.size());
  int result = udp.endPacket();

  // sendUdpPacket(uwbData, 10);

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
  int status;
  // if (accel.begin(Bmi088Accel::RANGE_12G, Bmi088Accel::ODR_200HZ_BW_80HZ) < 0 ||
  //     gyro.begin(Bmi088Gyro::RANGE_1000DPS, Bmi088Gyro::ODR_200HZ_BW_64HZ) < 0) {
  //   // Serial.println("Sensor initialization failed");
  //   while (1);
  // }

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
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
  IMU.setSrd(4); // Set sample rate divider to 19 for 50 Hz output data rate

    /* Start the sensors */
  status = bmi.begin(Bmi088::ACCEL_RANGE_12G, Bmi088::GYRO_RANGE_1000DPS, Bmi088::ODR_1000HZ);
  if (status < 0) {
    Serial.println("IMU Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  
  /* Set the ranges */
  status = bmi.setRange(Bmi088::ACCEL_RANGE_6G, Bmi088::GYRO_RANGE_1000DPS);
  if (status < 0) {
    Serial.println("Failed to set ranges");
    Serial.println(status);
    while (1) {}
  }
  
  /* Set the output data rate */
  status = bmi.setOdr(Bmi088::ODR_200HZ);
  if (status < 0) {
    Serial.println("Failed to set ODR");
    Serial.println(status);
    while (1) {}
  }

  /* Map pin 26 for accelerometer data ready interrupt */
  status = bmi.mapDrdy(Bmi088::PIN_2); // Assuming PIN_2 is mapped to pin 26
  if (status < 0) {
    Serial.println("Failed to map accelerometer data ready pin");
    Serial.println(status);
    while (1) {}
  }
  
  /* Map pin 33 for gyroscope sync interrupt */
  status = bmi.mapSync(Bmi088::PIN_4); // Assuming PIN_4 is mapped to pin 33
  if (status < 0) {
    Serial.println("Failed to map gyroscope sync pin");
    Serial.println(status);
    while (1) {}
  }
  
  /* Set the data ready pin to push-pull and active high */
  status = bmi.pinModeDrdy(Bmi088::PUSH_PULL, Bmi088::ACTIVE_HIGH);
  if (status < 0) {
    Serial.println("Failed to set up data ready pin");
    Serial.println(status);
    while (1) {}
  }

  /* Attach the corresponding uC pins to interrupts */
  pinMode(26, INPUT);
  attachInterrupt(26, imuInt, RISING);

//   pinMode(33, INPUT);
//   attachInterrupt(33, drdy, RISING); // Assuming you're using the same handler for both


  // Set up access point (AP) mode
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress AP_IP = WiFi.softAPIP();
  // Serial.print("AP SSID: ");
  // Serial.println(ap_ssid);
  // Serial.print("AP IP address: ");
  // Serial.println(AP_IP);
  
  // compass.setMagneticDeclination(54, 58); // Set magnetic declination for your location (degrees, minutes)

  compass.init();
  // compass.setMagneticDeclination(54, 58); // Set magnetic declination for your location (degrees, minutes)

    // Create the task for sending IMU data on Core 1
    // xTaskCreatePinnedToCore(
    //     sendIMUTask,     // Function to implement the task
    //     "SendIMUTask",   // Name of the task
    //     10000,           // Stack size in words
    //     NULL,            // Task input parameter
    //     1,               // Priority of the task
    //     &sendIMUTaskHandle, // Task handle
    //     0);              // Core where the task should run (Core 1)

    // Create the task for sending IMU data on Core 1
    // xTaskCreatePinnedToCore(
    //     sendIMUMAGTask,     // Function to implement the task
    //     "SendIMUMAGTask",   // Name of the task
    //     10000,           // Stack size in words
    //     NULL,            // Task input parameter
    //     1,               // Priority of the task
    //     &sendIMUMAGTaskHandle, // Task handle
    //     0);              // Core where the task should run (Core 1)


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
  // server.on("/rawSwitch", handleRawData);

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

    DW1000Ranging.loop();

    static unsigned long lastIMUSendTime = 0;
    static unsigned long lastRangeTime = 0;
    static unsigned long lastMAGSendTime = 0;
    static unsigned long lastIMUMAGSendTime = 0;
    static unsigned long last = 0;

    const unsigned long UWB_RATE = 15000;    // 15 ms interval for UWB data
    const unsigned long IMU_RATE = 5000;     // 5 ms interval for IMU data
    const unsigned long MAG_RATE = 5000;     // 5 ms interval for Magnetometer data

    // const unsigned long IMUMAG_RATE = 4100;  // 5 ms interval for combined IMU and Magnetometer data
    const unsigned long IMUMAG_RATE = 5000;  // 5 ms interval for combined IMU and Magnetometer data
    unsigned long currentTime = micros();  // Use micros() for higher resolution

    // if (sendData && currentTime - last >= 3000000) {  // Handle web requests every 3000 ms (3 seconds)
        server.handleClient();
    //     last = currentTime;
    // }

    static unsigned long lastCheckTime = 0;
    // static int lastCounter = 0;

    // unsigned long currentTime = millis();

    // Read sensor data
    // IMU.readSensor();

    // Extract raw data directly
    // accelXRaw = IMU.getAccelXRaw();
    // accelYRaw = IMU.getAccelYRaw();
    // accelZRaw = IMU.getAccelZRaw();
    
    // gyroXRaw = IMU.getGyroXRaw();
    // gyroYRaw = IMU.getGyroYRaw();
    // gyroZRaw = IMU.getGyroZRaw();

    // magXRaw = IMU.getMagXRaw();
    // magYRaw = IMU.getMagYRaw();
    // magZRaw = IMU.getMagZRaw();

    // Check if 1 second has passed
    // if (currentTime - lastCheckTime >= 1000) {
    //     // Disable interrupts while reading shared variable
    //     noInterrupts();
    //     int countInLastSecond = counter;
    //     int countInLastSecond2 =counter2;
    //     counter = 0;  // Reset the counter
    //     counter2 = 0;
    //     interrupts();

    //     // Serial.print("Interrupts in last UWB second: ");
    //     // Serial.println(countInLastSecond);
    //     // Serial.print("Interrupts in last IMU second: ");
    //     // Serial.println(countInLastSecond2);

    //     lastCheckTime = currentTime;
    // }

    switch (currentMode) {
    case IMU_ONLY:
        if (imuReady == 1) {
            imuReady = 0;
            detachInterrupt(digitalPinToInterrupt(34));
            bmi.readSensor();
            imuDataReadyToSend = true;
            attachInterrupt(digitalPinToInterrupt(34), DW1000Class::handleInterrupt, RISING);

            if (sendIMUTaskHandle == NULL) {
                xTaskCreatePinnedToCore(
                    sendIMUTask,     // Function to implement the task
                    "SendIMUTask",   // Name of the task
                    10000,           // Stack size in words
                    NULL,            // Task input parameter
                    1,               // Priority of the task
                    &sendIMUTaskHandle, // Task handle
                    0);              // Core 1
            }
        }
        break;

        case UWB_CALIBRATION:
            if (calibration_in_progress && currentTime - lastRangeTime >= UWB_RATE) {  // 15 ms interval for UWB data
                DW1000Ranging.loop();
                lastRangeTime = currentTime;
            }
            break;

        case UWB_DATA:
            // if (currentTime - lastRangeTime >= UWB_RATE) {  // 15 ms interval for UWB data
                DW1000Ranging.loop();
            //     lastRangeTime = currentTime;
            // }
            break;

        case IMU_UWB_DATA:
              if (imuReady == 1){
                      // Disable UWB interrupt while processing IMU data
                      detachInterrupt(digitalPinToInterrupt(34));
                      // noInterrupts();
                      // Serial.println(counter);
                        // Serial.println(Count);
                      bmi.readSensor();
                      // bmi.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw, &gyroX_raw, &gyroY_raw, &gyroZ_raw);
                      // noInterrupts();

                      // Set the flag to signal the task on Core 1 to send IMU data
                      imuDataReadyToSend = true;

                      // sendIMUData();
                      imuReady = 0;
                      // Re-enable UWB interrupt after IMU processing
                      attachInterrupt(digitalPinToInterrupt(34), DW1000Class::handleInterrupt, RISING);
    
                      // interrupts();
              }
            // if (currentTime - lastRangeTime >= UWB_RATE) {  // 15 ms interval for UWB data
                  DW1000Ranging.loop();
            //     lastRangeTime = currentTime;
            // }
            break;

        case COMPASS:
            if (sendData && currentTime - lastMAGSendTime >= MAG_RATE) {  // 5 ms interval for Magnetometer data
                sendCompass();
                lastMAGSendTime = currentTime;
            }
            break;

      case IMU_COMPASS:
          // compass.read();


          // int64_t startTime = esp_timer_get_time();  // Start timing

          // float tio_millis = static_cast<float>(millis());
          // Tio = tio_millis / 1000.0;
              // Get the elapsed time in nanoseconds
          // int64_t elapsedTimeNs = esp_timer_get_time() * 1000;  // Convert microseconds to nanoseconds
          // // myArray4[0] = Tio;
          // myArray4[1] = accelX_raw;
          // myArray4[2] = accelY_raw;
          // myArray4[3] = accelZ_raw;
          // myArray4[4] = gyroX_raw;
          // myArray4[5] = gyroY_raw;
          // myArray4[6] = gyroZ_raw;
          // myArray4[7] = compass.getX();
          // myArray4[8] = compass.getY();
          // myArray4[9] = compass.getZ();

          // Serial.print(compass.getX());
          // Serial.print(", ");
          // Serial.print(compass.getY());
          // Serial.print(", ");
          // Serial.println(compass.getZ());
          // sendIMU_MAG();
          // delay(2);
          if (sendData && currentTime - lastIMUMAGSendTime >= IMUMAG_RATE) {  // IMU data every 10ms

            lastIMUMAGSendTime = currentTime;

            sendIMU_MAG();
          }
          break;
          // if (imuReady == 1) {
          //     imuReady = 0;
          //     detachInterrupt(digitalPinToInterrupt(34));
              
          //     bmi.readSensor();
          //     // compass.read();
          //     imuMagDataReadyToSend = true;
          //     attachInterrupt(digitalPinToInterrupt(34), DW1000Class::handleInterrupt, RISING);

          //     if (sendIMUMAGTaskHandle == NULL) {
          //         xTaskCreatePinnedToCore(
          //             sendIMUMAGTask,     // Function to implement the task
          //             "SendIMUMAGTask",   // Name of the task
          //             10000,              // Stack size in words
          //             NULL,               // Task input parameter
          //             1,                  // Priority of the task
          //             &sendIMUMAGTaskHandle, // Task handle
          //             0);                 // Core 1
          //     }
          // }
          break;


    case IMU_UWB_COMPASS:
        if (imuReady == 1) {
            imuReady = 0;
            detachInterrupt(digitalPinToInterrupt(34));
            bmi.readSensor();
            compass.read();
            imuMagDataReadyToSend = true;
            attachInterrupt(digitalPinToInterrupt(34), DW1000Class::handleInterrupt, RISING);

            if (sendIMUMAGTaskHandle == NULL) {
                xTaskCreatePinnedToCore(
                    sendIMUMAGTask,     // Function to implement the task
                    "SendIMUMAGTask",   // Name of the task
                    10000,              // Stack size in words
                    NULL,               // Task input parameter
                    1,                  // Priority of the task
                    &sendIMUMAGTaskHandle, // Task handle
                    0);                 // Core 1
            }
        }

        // Ensure UWB data is processed
          DW1000Ranging.loop();
        break;

        // case IMU_ONLY_2:



        case Terminate:
          // Optionally delete tasks if switching out of states
          if (sendIMUTaskHandle != NULL) {
              vTaskDelete(sendIMUTaskHandle);
              sendIMUTaskHandle = NULL;
          }
          if (sendIMUMAGTaskHandle != NULL) {
              vTaskDelete(sendIMUMAGTaskHandle);
              sendIMUMAGTaskHandle = NULL;
        }


    default:
        // Optionally delete tasks if switching out of states
        if (sendIMUTaskHandle != NULL) {
            vTaskDelete(sendIMUTaskHandle);
            sendIMUTaskHandle = NULL;
        }
        if (sendIMUMAGTaskHandle != NULL) {
            vTaskDelete(sendIMUMAGTaskHandle);
            sendIMUMAGTaskHandle = NULL;
        }
        break;
            }
}
