#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include "MPU9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

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
// Variables to store raw sensor data
int16_t accelXRaw, accelYRaw, accelZRaw;
int16_t gyroXRaw, gyroYRaw, gyroZRaw;
int16_t magXRaw, magYRaw, magZRaw;
int16_t tempRaw;

// Constants
#define I2C_SDA 21
#define I2C_SCL 22
#define MPU9250_ADDRESS 0x68 // I2C address for MPU9250

// WiFi and UDP settings
const char* ap_ssid = "ESP32_AP";
const char* ap_password = "123456789";
const char* udpAddress = "192.168.4.100";  // Update as needed
const int udpPort = 12346;

// Define sensor data structure
struct SensorData {
    int64_t timestamp_ns;
    float accel[3];
    float gyro[3];
    float mag[3];
};

// FreeRTOS queue handle
QueueHandle_t sensorDataQueue;

// MPU9250 Object
MPU9250Extended IMU(Wire, 0x68);

// Web Server
WebServer server(80);

// UDP
WiFiUDP udp;

// Function Prototypes
void handleStart();
void handleStop();
void handleSetBatch();
void handleModeChange();
void SensorTask(void * parameter);
void SendTask(void * parameter);

// Mode Enumeration
enum Mode {
    IMU_ONLY = 5,
    Terminate = 99
};

Mode currentMode = IMU_ONLY;

// Task Handles
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t sendTaskHandle = NULL;

// Web Server Handlers
void handleStart() {
    server.send(200, "text/plain", "Data transmission started");
}

void handleStop() {
    server.send(200, "text/plain", "Data transmission stopped");
}

void handleSetBatch() {
    // Implement batch size handling if needed
    server.send(200, "text/plain", "Batch size set");
}

void handleModeChange() {
    if (server.hasArg("mode")) {
        int mode = server.arg("mode").toInt();
        if (mode == IMU_ONLY || mode == Terminate) {
            currentMode = static_cast<Mode>(mode);
            server.send(200, "text/plain", "Mode set");
        } else {
            server.send(400, "text/plain", "Invalid mode");
        }
    } else {
        server.send(400, "text/plain", "Mode not specified");
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // Initialize MPU9250
    if (IMU.begin() < 0) {
        Serial.println("MPU9250 initialization failed.");
        while (1);
    }
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_10HZ);
    IMU.setSrd(4); // Set sample rate divider if applicable

    // Initialize WiFi AP
    WiFi.softAP(ap_ssid, ap_password);
    IPAddress AP_IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(AP_IP);

    // Initialize UDP
    udp.begin(udpPort);
    Serial.print("UDP started on port ");
    Serial.println(udpPort);

    // Set up web server routes
    server.on("/start", handleStart);
    server.on("/stop", handleStop);
    server.on("/setBatch", handleSetBatch);
    server.on("/mode", handleModeChange);
    server.begin();
    Serial.println("HTTP server started");

    // Create FreeRTOS Queue
    sensorDataQueue = xQueueCreate(10, sizeof(SensorData)); // Queue length 10

    // Create FreeRTOS Tasks
    xTaskCreatePinnedToCore(
        SensorTask,       // Function to implement the task
        "SensorTask",     // Name of the task
        4096,             // Stack size in words
        NULL,             // Task input parameter
        1,                // Priority of the task
        &sensorTaskHandle,// Task handle
        0);               // Core where the task should run (Core 0)

    xTaskCreatePinnedToCore(
        SendTask,         // Function to implement the task
        "SendTask",       // Name of the task
        4096,             // Stack size in words
        NULL,             // Task input parameter
        1,                // Priority of the task
        &sendTaskHandle,  // Task handle
        1);               // Core where the task should run (Core 1)
}

void loop() {
    // Handle web server
    server.handleClient();

    // No other code needed here as tasks handle sensor and sending
}

// Sensor Task: Reads MPU9250 data at 200 Hz
void SensorTask(void * parameter) {
    const TickType_t frequency = 5 / portTICK_PERIOD_MS; // 200 Hz -> 5 ms
    TickType_t xLastWakeTime = xTaskGetTickCount();

    SensorData data;

    while (1) {
        // Read IMU data
        IMU.readSensor();
        data.accel[0] = IMU.getAccelX_mss();
        data.accel[1] = IMU.getAccelY_mss();
        data.accel[2] = IMU.getAccelZ_mss();

        data.gyro[0] = IMU.getGyroX_rads();
        data.gyro[1] = IMU.getGyroY_rads();
        data.gyro[2] = IMU.getGyroZ_rads();

        // // Read Magnetometer data
        // // Assuming MPU9250 class has getMagX_uT(), getMagY_uT(), getMagZ_uT()
        // // If not, you need to use an external magnetometer library or implement these methods
        data.mag[0] = IMU.getMagX_uT();
        data.mag[1] = IMU.getMagY_uT();
        data.mag[2] = IMU.getMagZ_uT();

        // data.accel[0] = accelXRaw;
        // data.accel[1] = accelYRaw;
        // data.accel[2] = accelZRaw;

        // data.gyro[0] = gyroXRaw;
        // data.gyro[1] = gyroYRaw;
        // data.gyro[2] = gyroZRaw;

        // // // Read Magnetometer data
        // // // Assuming MPU9250 class has getMagX_uT(), getMagY_uT(), getMagZ_uT()
        // // // If not, you need to use an external magnetometer library or implement these methods
        // data.mag[0] = magXRaw;
        // data.mag[1] = magYRaw;
        // data.mag[2] = magZRaw;

        // Get timestamp
        data.timestamp_ns = esp_timer_get_time() * 1000; // Convert microseconds to nanoseconds

        // Send data to queue
        if (xQueueSend(sensorDataQueue, &data, (TickType_t)0) != pdPASS) {
            // Queue is full, handle overflow
            Serial.println("SensorDataQueue full. Data lost.");
        }

        // Wait until next cycle
        vTaskDelayUntil(&xLastWakeTime, frequency);
    }
}

// Send Task: Sends sensor data via UDP on Core 1
void SendTask(void * parameter) {
    SensorData receivedData;
    union {
        float floatingPoint;
        uint8_t binary[4];
    } bf;

    while (1) {
        // Wait for data from queue
        if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdTRUE) {
            // Prepare UDP packet
            std::vector<uint8_t> udpPacket;

            // Add separator
            const char* IMU_MAG_SEPARATOR = "img/";
            udpPacket.insert(udpPacket.end(), IMU_MAG_SEPARATOR, IMU_MAG_SEPARATOR + strlen(IMU_MAG_SEPARATOR));

            // Add timestamp
            uint8_t* timestampPtr = reinterpret_cast<uint8_t*>(&receivedData.timestamp_ns);
            udpPacket.insert(udpPacket.end(), timestampPtr, timestampPtr + sizeof(int64_t));

            // Add accelerometer data
            for (int i = 0; i < 3; i++) {
                bf.floatingPoint = receivedData.accel[i];
                udpPacket.insert(udpPacket.end(), bf.binary, bf.binary + 4);
            }

            // Add gyroscope data
            for (int i = 0; i < 3; i++) {
                bf.floatingPoint = receivedData.gyro[i];
                udpPacket.insert(udpPacket.end(), bf.binary, bf.binary + 4);
            }

            // Add magnetometer data
            for (int i = 0; i < 3; i++) {
                bf.floatingPoint = receivedData.mag[i];
                udpPacket.insert(udpPacket.end(), bf.binary, bf.binary + 4);
            }

            // Send UDP packet
            udp.beginPacket(udpAddress, udpPort);
            udp.write(udpPacket.data(), udpPacket.size());
            udp.endPacket();
        }
    }
}
