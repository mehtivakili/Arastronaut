# Arastronaut: Open-Source Indoor Positioning Platform

![Arastronaut Logo](https://github.com/mehtivakili/Arastronaut/blob/main/assets/menu_page-0001%20(Small).jpg) <!-- Placeholder for Image 1 -->

**Arastronaut** is a cutting-edge open-source platform that combines hardware and software to deliver high-precision indoor positioning. Designed for researchers, developers, and enthusiasts, this project leverages Ultra-Wideband (UWB) technology and Inertial Measurement Units (IMUs) to achieve centimeter-level accuracy in diverse applications.

---

## Key Features

### Hardware Components
- **ESP32 Microcontroller**: A cost-effective, dual-core processor with built-in Wi-Fi capabilities.
- **DWM1000 UWB Transceiver**: Provides precise distance measurements using Time of Flight (ToF) calculations.
- **BMI088 IMU**: A robust inertial sensor for detecting linear acceleration and angular velocity.
- **Additional Sensors**:
  - QMC5883L Magnetometer for accurate heading estimation.
  - BMP280 Barometric Pressure Sensor for 3D localization.
- **User-Friendly OLED Display**: For monitoring system parameters and debugging.

### Software Highlights
- **Two-Way Ranging (TWR)**: A reliable distance measurement method that eliminates the need for strict synchronization.
- **Time Division Multiple Access (TDMA)**: Ensures robust synchronization and prevents signal collisions.
- **Real-Time Operating System (RTOS)**: Optimized for dynamic environments with high data throughput.
- **Graphical User Interface (GUI)**: Simplifies calibration, monitoring, and data collection.

---

## Use Cases
- Indoor navigation for robots and drones.
- Motion tracking in AR/VR systems.
- Asset tracking in warehouses and industrial facilities.
- Research on sensor fusion and localization techniques.

---

## Getting Started

### Prerequisites
- ESP32 development board.
- DWM1000 UWB modules and supporting sensors.
- USB cables for firmware flashing.
- Python environment (for GUI and data processing).

### Installation
1. **Clone the Repository**:
    ```bash
    git clone https://github.com/mehtivakili/Arastronaut.git
    cd Arastronaut
    ```

2. **Install Python Dependencies**:
    ```bash
    pip install -r requirements.txt
    ```

3. **Flash Firmware**:
    - Connect the ESP32 to your computer.
    - Use the GUI or ESP32 flashing tools to upload the firmware.

4. **Setup Anchors and Tags**:
    - Configure anchor positions and assign unique IDs.
    - Define the number of anchors needed based on your environment.

---

## System Architecture

### Hardware Overview
![System Architecture](https://github.com/mehtivakili/Arastronaut/blob/main/assets/Screenshot%202024-11-26%20084929%20(Small)%20(Small).png) <!-- Placeholder for Image 2 -->

The platform consists of modular components for easy deployment and scalability. Anchors are strategically placed in fixed locations, while the mobile tag computes its position using trilateration. 

- Anchors: Fixed reference points using DWM1000 modules.
- Tags: Mobile units equipped with UWB, IMU, and other sensors.

### Software Layers
1. **Firmware**:
   - Configures hardware and establishes communication protocols.
   - Implements TDMA for time slot management.
2. **Calibration**:
   - Magnetometer: Corrects hard/soft iron distortions.
   - IMU: Adjusts for biases and misalignment.
   - UWB: Accounts for antenna delays.
3. **Sensor Fusion**:
   - Combines UWB and IMU data for enhanced accuracy.
   - Processes real-time motion data for dynamic environments.

---

## Graphical User Interface (GUI)

![GUI Features](https://github.com/mehtivakili/Arastronaut/blob/main/assets/ezgif-6-539f7a2398.gif) <!-- Placeholder for Image 3 -->

The GUI is designed for user-friendliness and functionality:
- **IMU Calibration**:
  - Displays raw and calibrated data for accelerometer and gyroscope.
  - Provides tools for parameter adjustments and visualization.
- **UWB Pre-Calibration**:
  - Allows users to input anchor positions and visualize localization results.
- **Magnetometer Calibration**:
  - Compensates for sensor errors to achieve accurate heading estimation.
- **Data Monitoring**:
  - Real-time plots for sensor output.
  - Export options for further analysis.

---

## Experimental Validation

![Verification Animation](https://github.com/mehtivakili/Arastronaut/blob/main/assets/Figure%201%202024-11-04%2022-43-26%20(1).gif) <!-- Placeholder for GIF -->

### IMU Accuracy Validation
The IMU was tested on a precision rotary table to compare its output against a reference encoder. Results showed a **Root Mean Square Error (RMSE)** of 2 degrees for orientation estimation.
![IMU](https://github.com/mehtivakili/Arastronaut/blob/main/assets/imu.jpg) <!-- Placeholder for Image 3-->


### UWB Ranging Analysis
Two DWM1000 modules were tested in a controlled environment. Linear regression modeling reduced the RMSE from **15 cm to 6 cm**, showcasing the system’s high accuracy.
![UWB](https://github.com/mehtivakili/Arastronaut/blob/main/assets/uwb.jpg) <!-- Placeholder for Image 4 -->

---

## Future Directions
- Integration of advanced sensor fusion algorithms.
- Exploration of Non-Line-of-Sight (NLOS) mitigation techniques.
- Adaptation for larger-scale environments and multi-floor navigation.
- Development of an API for third-party integrations.

---

## Contributing

Contributions are welcome! To contribute:
1. Fork the repository.
2. Create a feature branch:
    ```bash
    git checkout -b feature-name
    ```
3. Submit a pull request with a detailed description of your changes.

For major changes, please open an issue to discuss your proposed modifications.

---

## Support

If you encounter any issues or have questions, please open an [issue](https://github.com/mehtivakili/Arastronaut/issues) in this repository or contact us directly.

---

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for more details.

---

## References
This project is based on the paper:  
_Vakili, M., Mirjalili, A. S., Dindarloo, M. R., Sharifi, A., Taghirad, H. D., “Arastronaut: An Open Source UWB/IMU Hardware and Software for Indoor Positioning”_, Proceedings of ICRoM 2024.

