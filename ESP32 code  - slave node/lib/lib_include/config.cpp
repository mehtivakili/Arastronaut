#include "config.h"

// WiFi credentials
const char *ssid = "your_SSID";
const char *password = "your_PASSWORD";

// Server settings
const char *host = "your_HOST";
const uint16_t portNum = 1234;

// UWB settings
bool UWB_PRECALIBRATION = false;
int DEVICE_TYPE = 1; // 1 for TAG, 0 for ANCHOR

// UWB antenna delay parameters
float tADTX = 515; // Initial transmit antenna delay
float tADRX = 515; // Initial receive antenna delay

// Define tag and anchor addresses
char tag_addr[] = "02:00:00:00:00:00:02:02"; // Tag address
char anchor_addr[] = "02:00:00:00:00:00:02:03"; // Anchor address

// Calibration parameters
bool IMU_CALIBRATED = false;
float Ta[3][3] = {{1, -0.00546066, 0.00101399}, {0, 1, 0.00141895}, {0, 0, 1}};
float Ka[3][3] = {{0.00358347, 0, 0}, {0, 0.00358133, 0}, {0, 0, 0.00359205}};
float acce_bias[3] = {-8.28051, -4.6756, -0.870355};
float Tg[3][3] = {{1, -0.00614889, -0.000546488}, {0.0102258, 1, 0.000838491}, {0.00412113, 0.0020154, 1}};
float Kg[3][3] = {{0.000531972, 0, 0}, {0, 0.000531541, 0}, {0, 0, 0.000531}};
float gyro_bias[3] = {4.53855, 4.001, -1.9779};

// Distances
float actual_distance = 7.19; // Example distance
float initial_measured_distance = 7.19; // Example initial measured distance
