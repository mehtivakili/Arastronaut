# ESP32 IMU and Gyroscope Data Collection and Visualization

This repository contains code for collecting and visualizing IMU and gyroscope data from an ESP32 microcontroller using Python, C++, and Node.js. The system includes data acquisition, calibration, and real-time plotting features.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Configuration](#configuration)
- [Endpoints](#endpoints)
- [Calibration](#calibration)
- [Real-Time Plotting](#real-time-plotting)
- [License](#license)

## Features

- **Data Acquisition**: Collect data from the ESP32's IMU and gyroscope sensors.
- **Calibration**: Apply calibration parameters to the raw sensor data.
- **Real-Time Plotting**: Visualize the sensor data in real-time using Plotly and Chart.js.
- **Web Interface**: Control data collection and view plots through a web interface.
- **Socket.IO Integration**: Stream data to a Node.js server for real-time updates.
- 
## AHRS quaternion and screw representation

![Demo GIF](assets/screw_representation_small.gif)

## Installation

### Prerequisites

- Python 3.x
- Flask
- Flask-SocketIO
- Node.js
- ESP32 development environment (Arduino or PlatformIO)
- Required Python packages:
  ```bash
  pip install flask flask-socketio flask-cors esptool serial matplotlib pandas
  ```

### Clone the Repository

```bash
git clone https://github.com/mehtivakil/arastronaut
```

### Setting up the ESP32

1. Open the provided C++ code in your preferred ESP32 development environment.
2. Adjust the WiFi credentials and other configurations as necessary.
3. Upload the code to your ESP32.

### Setting up the Flask Server

1. Navigate to the `flask_server` directory.
2. Run the Flask server:
   ```bash
   python app.py
   ```

### Setting up the Node.js Server

1. Navigate to the `node_server` directory.
2. Install the required Node.js packages:
   ```bash
   npm install
   ```
3. Start the Node.js server:
   ```bash
   node server.js
   ```

## Usage

### Accessing the Web Interface

1. Open your browser and navigate to `http://<ESP32_IP>:81` to access the ESP32's web interface.
2. Use the Flask server's web interface at `http://127.0.0.1:5000` for additional features like calibration and data plotting.

### Real-Time Plotting

- Use the web interface to start data collection.
- View real-time plots of acceleration and gyroscope data on the web interface.

### Calibration

- Navigate to the calibration page and upload your calibration parameters.
- Apply the calibration to your sensor data for more accurate measurements.

## Project Structure

```bash
.
├── flask_server
│   ├── app.py
│   ├── templates
│   │   ├── index.html
│   │   ├── firmware.html
│   │   ├── data_acquisition.html
│   │   └── calibration.html
│   ├── static
│   │   ├── css
│   │   └── js
├── esp32_code
│   ├── main.cpp
│   ├── config2.h
│   └── NetworkConsole.cpp
├── node_server
│   ├── index.html
│   ├── server.js
│   └── package.json
└── README.md
```

## Configuration

### Flask Server

- `FIRMWARE_BASE_PATH`: Path to the firmware files for flashing.
- Serial port settings and calibration parameters.

### ESP32 Code

- WiFi credentials and server IP settings.
- Calibration matrices and biases.

## Endpoints

### Flask Server

- `/`: Home page displaying network information.
- `/firmware`: Firmware flashing page.
- `/data_acquisition`: Data acquisition page.
- `/calibration`: Calibration page.
- `/python_serial`: Serial port configuration and data handling.
- `/flash`: Firmware flashing endpoint.
- `/open_serial`: Open serial port.
- `/close_serial`: Close serial port.
- `/start_recording`: Start data recording.
- `/stop_recording`: Stop data recording.
- `/plot_data`: Plot recorded data.
- `/calibrate`: Perform calibration.
- `/get_calibrated_data`: Retrieve calibrated data.
- `/upload_calibration_files`: Upload calibration files.

## Calibration

1. Upload your calibration files through the web interface.
2. Apply the calibration parameters to the raw sensor data.

## Real-Time Plotting

- Visualize acceleration and gyroscope data in real-time.
- Use the provided HTML and JavaScript code for dynamic updates.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

Feel free to contribute to this project by submitting issues or pull requests. For any questions, please contact [metiva76@gmail.com](mailto:metiva76@gmail.com).
