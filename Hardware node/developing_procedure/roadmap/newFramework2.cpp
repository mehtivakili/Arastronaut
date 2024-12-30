#include <WiFi.h>
#include <WebServer.h>
#include <WiFiUdp.h>

// WiFi and UDP setup
WiFiUDP udp;
WebServer server(80);
const char* udpAddress = "192.168.1.100"; // Replace with your receiver's IP
const int udpPort = 12345;

// System start time for timestamping
unsigned long systemStartTime;

// Flags for data readiness
bool imuReady = false;
bool magReady = false;
bool uwbReady = false;
bool baroReady = false;

// Buffer for UDP data
char udpBuffer[512]; // Adjust size as needed

// Enum for operation modes
enum OperationMode {
    MODE_IMU_ONLY,
    MODE_MAG_ONLY,
    MODE_UWB_ONLY,
    MODE_BARO_ONLY,
    MODE_IMU_UWB,
    MODE_IMU_MAG,
    MODE_IMU_BARO,
    MODE_MAG_UWB,
    MODE_MAG_BARO,
    MODE_UWB_BARO,
    MODE_IMU_MAG_UWB,
    MODE_IMU_MAG_BARO,
    MODE_IMU_UWB_BARO,
    MODE_MAG_UWB_BARO,
    MODE_IMU_MAG_UWB_BARO,
    MODE_ALL_SENSORS
};

enum TransmissionMethod {
    TRANSMIT_UDP,
    TRANSMIT_SERIAL
};

TransmissionMethod currentTransmissionMethod = TRANSMIT_UDP; // Default method
int serialBaudRate = 115200; // Default baud rate


OperationMode currentMode = MODE_IMU_ONLY; // Default mode
const unsigned long batchInterval = 10; // 10 ms interval for sending data
unsigned long lastBatchTime = 0;

void setup() {
    // Initialize serial communication (optional for debugging)
    Serial.begin(115200);

    // Initialize Wi-Fi
    WiFi.begin("yourSSID", "yourPASSWORD");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");

    // Start UDP
    udp.begin(udpPort);

    // Initialize HTTP server routes
    server.on("/", handleRoot);
    server.on("/setmode", handleSetMode);
    server.begin();

    // Add these routes to your existing server setup
    server.on("/settransmission", handleSetTransmission);
    server.on("/setbaudrate", handleSetBaudRate);

    // Initialize sensors
    initIMU();
    initMagnetometer();
    initUWB();
    initBarometer();

    // Record system start time
    systemStartTime = millis();
}

void loop() {
    unsigned long currentTime = millis();

    // Prepare data when it's ready (non-blocking)
    if (dataIMUReady()) {
        prepareIMUData();
    }
    if (dataMagReady()) {
        prepareMagData();
    }
    if (dataUWBReady()) {
        prepareUWBData();
    }
    if (dataBaroReady()) {
        prepareBaroData();
    }

    // Send the entire batch every 10 ms
    if (currentTime - lastBatchTime >= batchInterval) {
        lastBatchTime = currentTime;
        sendDataBatch();
    }

    server.handleClient(); // Handle HTTP requests
}

void handleRoot() {
    server.send(200, "text/plain", "Use /setmode?mode=X to set mode");
}

// Handling the transmission method setting
void handleSetTransmission() {
    if (server.hasArg("method")) {
        String method = server.arg("method");
        if (method == "udp") {
            currentTransmissionMethod = TRANSMIT_UDP;
            server.send(200, "text/plain", "Transmission method set to UDP");
        } else if (method == "serial") {
            currentTransmissionMethod = TRANSMIT_SERIAL;
            server.send(200, "text/plain", "Transmission method set to Serial");
        } else {
            server.send(400, "text/plain", "Invalid transmission method");
        }
    } else {
        server.send(400, "text/plain", "Transmission method parameter missing");
    }
}

// Handling the baud rate setting
void handleSetBaudRate() {
    if (server.hasArg("baudrate")) {
        int rate = server.arg("baudrate").toInt();
        if (rate == 9600 || rate == 19200 || rate == 38400 || rate == 57600 || rate == 115200 || rate == 921600) {
            serialBaudRate = rate;
            Serial.begin(serialBaudRate);  // Reinitialize Serial with the new baud rate
            server.send(200, "text/plain", "Baud rate set to " + String(rate));
        } else {
            server.send(400, "text/plain", "Invalid baud rate");
        }
    } else {
        server.send(400, "text/plain", "Baud rate parameter missing");
    }
}

void handleSetMode() {
    if (server.hasArg("mode")) {
        int mode = server.arg("mode").toInt();
        if (mode >= 0 && mode < 16) { // 16 modes in total
            currentMode = static_cast<OperationMode>(mode);
            server.send(200, "text/plain", "Mode set to " + String(mode));
        } else {
            server.send(400, "text/plain", "Invalid mode value");
        }
    } else {
        server.send(400, "text/plain", "Mode parameter missing");
    }
}

void handleSensors() {
    switch (currentMode) {
        case MODE_IMU_ONLY:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            break;
        case MODE_MAG_ONLY:
            if (dataMagReady()) {
                prepareMagData();
            }
            break;
        case MODE_UWB_ONLY:
            if (dataUWBReady()) {
                prepareUWBData();
            }
            break;
        case MODE_BARO_ONLY:
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
        case MODE_IMU_UWB:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            if (dataUWBReady()) {
                prepareUWBData();
            }
            break;
        case MODE_IMU_MAG:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            if (dataMagReady()) {
                prepareMagData();
            }
            break;
        case MODE_IMU_BARO:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
        case MODE_MAG_UWB:
            if (dataMagReady()) {
                prepareMagData();
            }
            if (dataUWBReady()) {
                prepareUWBData();
            }
            break;
        case MODE_MAG_BARO:
            if (dataMagReady()) {
                prepareMagData();
            }
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
        case MODE_UWB_BARO:
            if (dataUWBReady()) {
                prepareUWBData();
            }
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
        case MODE_IMU_MAG_UWB:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            if (dataMagReady()) {
                prepareMagData();
            }
            if (dataUWBReady()) {
                prepareUWBData();
            }
            break;
        case MODE_IMU_MAG_BARO:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            if (dataMagReady()) {
                prepareMagData();
            }
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
        case MODE_IMU_UWB_BARO:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            if (dataUWBReady()) {
                prepareUWBData();
            }
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
        case MODE_MAG_UWB_BARO:
            if (dataMagReady()) {
                prepareMagData();
            }
            if (dataUWBReady()) {
                prepareUWBData();
            }
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
        case MODE_IMU_MAG_UWB_BARO:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            if (dataMagReady()) {
                prepareMagData();
            }
            if (dataUWBReady()) {
                prepareUWBData();
            }
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
        case MODE_ALL_SENSORS:
            if (dataIMUReady()) {
                prepareIMUData();
            }
            if (dataMagReady()) {
                prepareMagData();
            }
            if (dataUWBReady()) {
                prepareUWBData();
            }
            if (dataBaroReady()) {
                prepareBaroData();
            }
            break;
    }
}

void prepareIMUData() {
    unsigned long timestamp = millis() - systemStartTime;
    strcat(udpBuffer, "imu/");
    strcat(udpBuffer, String(timestamp).c_str());
    strcat(udpBuffer, ",data1,data2,data3|"); // Replace with actual IMU data
}

void prepareMagData() {
    unsigned long timestamp = millis() - systemStartTime;
    strcat(udpBuffer, "mag/");
    strcat(udpBuffer, String(timestamp).c_str());
    strcat(udpBuffer, ",data4,data5,data6|"); // Replace with actual Magnetometer data
}

void prepareUWBData() {
    unsigned long timestamp = millis() - systemStartTime;
    strcat(udpBuffer, "uwb/");
    strcat(udpBuffer, String(timestamp).c_str());
    strcat(udpBuffer, ",data7,data8,data9|"); // Replace with actual UWB data
}

void prepareBaroData() {
    unsigned long timestamp = millis() - systemStartTime;
    strcat(udpBuffer, "baro/");
    strcat(udpBuffer, String(timestamp).c_str());
    strcat(udpBuffer, ",data10|"); // Replace with actual Barometer data
}

void sendDataBatch() {
    if (currentTransmissionMethod == TRANSMIT_UDP) {
        udp.beginPacket(udpAddress, udpPort);
        udp.write(udpBuffer);
        udp.endPacket();
    } else if (currentTransmissionMethod == TRANSMIT_SERIAL) {
        Serial.write(udpBuffer);
    }

    // Clear the buffer after sending
    memset(udpBuffer, 0, sizeof(udpBuffer));
}


// Example placeholder function to simulate UWB data availability
bool isUWBDataAvailable() {
    // Implement your logic to check if UWB data is available
    return true; // Simulate data availability for example purposes
}

void initIMU() {
    // Initialization code for IMU
}

void initMagnetometer() {
    // Initialization code for Magnetometer
}

void initUWB() {
    // Initialization code for UWB
}

void initBarometer() {
    // Initialization code for Barometer
}
