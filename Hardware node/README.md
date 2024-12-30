
This directory is intended for PlatformIO Test Runner and project tests.

Unit Testing is a software testing method by which individual units of
source code, sets of one or more MCU program modules together with associated
control data, usage procedures, and operating procedures, are tested to
determine whether they are fit for use. Unit testing finds problems early
in the development cycle.

More information about PlatformIO Unit Testing:
- https://docs.platformio.org/en/latest/advanced/unit-testing/index.html

# ESP32 UWB Indoor Localization with Web Configuration

This repository provides code for indoor localization using ESP32 UWB (DW1000) tags and anchors. It includes web configuration for real-time parameter adjustment.

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Setup and Installation](#setup-and-installation)
- [Libraries Used](#libraries-used)
- [Configuration](#configuration)
- [Web Interface](#web-interface)
- [Calibration](#calibration)
- [License](#license)

## Overview

This project aims to achieve indoor localization with ESP32 UWB modules. It supports real-time configuration through a web interface, enabling easy adjustments of parameters such as UWB pre-calibration, IMU calibration, and update intervals.

## Features

- Indoor localization using UWB technology
- Real-time configuration via a web interface
- Support for both TAG and ANCHOR devices
- IMU calibration with configurable parameters
- OTA updates

## Hardware Requirements

- ESP32 UWB modules from Makerfabs: [ESP32 UWB Ultra Wideband](https://www.makerfabs.com/esp32-uwb-ultra-wideband.html)
- At least four modules for 2D localization, five for 3D localization

## Setup and Installation

1. Clone the repository:

    ```sh
    [git clone](https://github.com/mehtivakili/iu.git)
    cd iu
    ```

2. Install PlatformIO IDE:

    [PlatformIO Installation Guide](https://platformio.org/install/ide?install=vscode)

3. Open the project in PlatformIO IDE.

4. Configure your WiFi credentials and other settings in `config.h`:

    ```cpp
    // config.h
    #ifndef CONFIG_H
    #define CONFIG_H

    #include <stdint.h>

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
    ```

5. Build and upload the code to your ESP32 modules.

## Libraries Used

- [ArduinoJson](https://arduinojson.org/)
- [WiFi](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi)
- [SPI](https://github.com/espressif/arduino-esp32/tree/master/libraries/SPI)
- [lcdgfx](https://github.com/lexus2k/lcdgfx)
- [Bolder Flight Systems BMI088](https://github.com/bolderflight/BMI088)
- [DW1000](https://github.com/thotro/arduino-dw1000) (customized)

## Configuration

The main configuration parameters are located in `config.h`. Below is an overview of the parameters:

- `UWB_PRECALIBRATION`: Enables or disables UWB pre-calibration.
- `DEVICE_TYPE`: Sets the device type (1 for TAG, 0 for ANCHOR).
- `updateInterval`: Interval for updates.
- `tADTX`, `tADRX`: UWB antenna delay parameters.
- `IMU_CALIBRATED`: Enables or disables IMU calibration.
- `Ta`, `Ka`, `acce_bias`, `Tg`, `Kg`, `gyro_bias`: IMU calibration parameters.
- `actual_distance`, `initial_measured_distance`: Distances for calibration.

## Web Interface

![Screenshot (180)](https://github.com/mehtivakili/iu/assets/36546765/5dd2f83e-7647-453c-86a0-8497d68edb5a)


The web interface allows real-time configuration of the parameters. Navigate to the IP address of your ESP32 in a web browser to access the interface.

### Example Web Interface Code

```html
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Configuration</title>
</head>
<body>
    <h1>Configuration</h1>
    <form action="/config" method="post">
        <label for="UWB_PRECALIBRATION">UWB Pre-calibration:</label>
        <input type="checkbox" id="UWB_PRECALIBRATION" name="UWB_PRECALIBRATION" checked><br><br>

        <label for="DEVICE_TYPE">Device Type (1 for TAG, 0 for ANCHOR):</label>
        <input type="number" id="DEVICE_TYPE" name="DEVICE_TYPE" value="1"><br><br>

        <label for="updateInterval">Update Interval:</label>
        <input type="number" id="updateInterval" name="updateInterval" value="200"><br><br>

        <label for="IMU_CALIBRATED">IMU Calibrated:</label>
        <input type="checkbox" id="IMU_CALIBRATED" name="IMU_CALIBRATED"><br><br>

        <label for="actual_distance">Actual Distance:</label>
        <input type="number" id="actual_distance" name="actual_distance" value="7.19"><br><br>

        <h2>IMU Calibration Parameters</h2>
        <!-- Add inputs for Ta, Ka, Tg, Kg, acce_bias, gyro_bias here -->

        <input type="submit" value="Set Configuration">
    </form>
    <h2>Console Output</h2>
    <pre id="console"></pre>
    <script>
        setInterval(function() {
            fetch('/console')
            .then(response => response.text())
            .then(data => {
                document.getElementById('console').textContent = data;
            });
        }, 1000);
    </script>
</body>
</html>
