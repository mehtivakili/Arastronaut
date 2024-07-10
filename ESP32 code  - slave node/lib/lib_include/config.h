#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h> // Include this header for uint16_t

// WiFi credentials
extern const char *ssid;
extern const char *password;

// Server settings
extern const char *host;
extern const uint16_t portNum;

// UWB settings
extern bool UWB_PRECALIBRATION;
extern int DEVICE_TYPE;
extern unsigned long updateInterval;

// UWB antenna delay parameters
extern float tADTX;
extern float tADRX;

// Define tag and anchor addresses
extern char tag_addr[];
extern char anchor_addr[];

// Calibration parameters
extern bool IMU_CALIBRATED;
extern float Ta[3][3];
extern float Ka[3][3];
extern float acce_bias[3];
extern float Tg[3][3];
extern float Kg[3][3];
extern float gyro_bias[3];

// Distances
extern float actual_distance;
extern float initial_measured_distance;

#endif // CONFIG_H
