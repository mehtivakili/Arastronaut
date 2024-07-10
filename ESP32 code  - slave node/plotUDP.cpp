#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "BMI088.h"
#include "config2.h"
#include <NetworkConsole.h>

WiFiManager wifiManager;
WiFiServer tcpServer(23);  // TCP server on port 23
NetworkConsole networkConsole;

Bmi088Accel accel(SPI, 32);
Bmi088Gyro gyro(SPI, 25);

int16_t accelX_raw, accelY_raw, accelZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;

float acce_calibrated[3];
float gyro_calibrated[3];

double Tio = 0.0;

const char* plotPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 IMU and Gyro Plotter</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin-top: 20px; }
    canvas { display: inline-block; margin: 20px; }
  </style>
</head>
<body>
  <h1>ESP32 IMU and Gyroscope Real-Time Plotter</h1>
  <div>
    <canvas id="accelChart" width="400" height="200"></canvas>
    <canvas id="gyroChart" width="400" height="200"></canvas>
  </div>
  <script>
    let accelChart, gyroChart;

    document.addEventListener("DOMContentLoaded", function() {
      const accelCtx = document.getElementById('accelChart').getContext('2d');
      const gyroCtx = document.getElementById('gyroChart').getContext('2d');

      accelChart = new Chart(accelCtx, {
        type: 'line',
        data: {
          labels: [],
          datasets: [
            { label: 'Accel X', borderColor: 'red', data: [] },
            { label: 'Accel Y', borderColor: 'green', data: [] },
            { label: 'Accel Z', borderColor: 'blue', data: [] }
          ]
        },
        options: {
          scales: {
            x: { type: 'linear', title: { text: 'Time (s)', display: true }},
            y: { min: -15, max: 15, title: { text: 'Acceleration (m/s²)', display: true }}
          }
        }
      });

      gyroChart = new Chart(gyroCtx, {
        type: 'line',
        data: {
          labels: [],
          datasets: [
            { label: 'Gyro X', borderColor: 'red', data: [] },
            { label: 'Gyro Y', borderColor: 'green', data: [] },
            { label: 'Gyro Z', borderColor: 'blue', data: [] }
          ]
        },
        options: {
          scales: {
            x: { type: 'linear', title: { text: 'Time (s)', display: true }},
            y: { min: -5, max: 5, title: { text: 'Rotation Rate (rad/s)', display: true }}
          }
        }
      });

      setInterval(fetchIMUData, 1000);
    });

    function fetchIMUData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          const time = data.time;

          accelChart.data.labels.push(time);
          gyroChart.data.labels.push(time);

          accelChart.data.datasets[0].data.push(data.accel[0]);
          accelChart.data.datasets[1].data.push(data.accel[1]);
          accelChart.data.datasets[2].data.push(data.accel[2]);

          gyroChart.data.datasets[0].data.push(data.gyro[0]);
          gyroChart.data.datasets[1].data.push(data.gyro[1]);
          gyroChart.data.datasets[2].data.push(data.gyro[2]);

          if (accelChart.data.labels.length > 200) {
            accelChart.data.labels.shift();
            gyroChart.data.labels.shift();
            accelChart.data.datasets.forEach(dataset => dataset.data.shift());
            gyroChart.data.datasets.forEach(dataset => dataset.data.shift());
          }

          accelChart.update();
          gyroChart.update();
        })
        .catch(error => console.error('Error fetching IMU data:', error));
    }
  </script>
</body>
</html>
)rawliteral";


void sendIMUData(WiFiClient client) {
    StaticJsonDocument<256> doc;
    doc["time"] = Tio;
    JsonArray accel = doc.createNestedArray("accel");
    accel.add(acce_calibrated[0]);
    accel.add(acce_calibrated[1]);
    accel.add(acce_calibrated[2]);

    JsonArray gyro = doc.createNestedArray("gyro");
    gyro.add(gyro_calibrated[0]);
    gyro.add(gyro_calibrated[1]);
    gyro.add(gyro_calibrated[2]);

    String output;
    serializeJson(doc, output);

    client.println(output);

    // Serial.println("Sending IMU Data: ");
    Serial.println(output);
}

void setup() {
    Serial.begin(115200);

    wifiManager.begin();

    // Start the network console
    networkConsole.begin();
    if (accel.begin(Bmi088Accel::RANGE_12G, Bmi088Accel::ODR_1600HZ_BW_234HZ) < 0 ||
        gyro.begin(Bmi088Gyro::RANGE_1000DPS, Bmi088Gyro::ODR_2000HZ_BW_230HZ) < 0) {
        Serial.println("Sensor initialization failed");
        while (1);
    }

    accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
    gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

    tcpServer.begin();  // Start the TCP server
    Serial.println("TCP server started");

    // Serve the plot page
    wifiManager.getServer().on("/", []() {
        wifiManager.getServer().send(200, "text/html", plotPage);
    });
    wifiManager.getServer().on("/plot", []() {
        wifiManager.getServer().send(200, "text/html", plotPage);
    });

wifiManager.getServer().on("/data", []() {
    WiFiClient client = tcpServer.available();
    if (client) {
        sendIMUData(client);
    }
});

    wifiManager.getServer().begin();
    Serial.println("HTTP server started");
}

void loop() {
    wifiManager.handleClient();
    ArduinoOTA.handle(); // Handle OTA updates

    accel.readSensor();
    gyro.readSensor();

    accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
    gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

    acce_calibrated[0] = ((Ka[0][0] * Ta[0][0]) + (Ka[0][1] * Ta[1][1]) + (Ka[0][2] * Ta[2][2])) * (accelX_raw - acce_bias[0]);
    acce_calibrated[1] = ((Ka[1][1] * Ta[1][1]) + (Ka[1][2] * Ta[2][2])) * (accelY_raw - acce_bias[1]);
    acce_calibrated[2] = ((Ka[2][2] * Ta[2][2])) * (accelZ_raw - acce_bias[2]);
    gyro_calibrated[0] = ((Kg[0][0] * Tg[0][0]) + (Kg[0][1] * Tg[1][1]) + (Kg[0][2] * Tg[2][2])) * (gyroX_raw - gyro_bias[0]);
    gyro_calibrated[1] = ((Kg[1][0] * Tg[1][0]) + (Kg[1][1] * Tg[1][1]) + (Kg[1][2] * Tg[2][2])) * (gyroY_raw - gyro_bias[1]);
    gyro_calibrated[2] = ((Kg[2][0] * Tg[2][0]) + (Kg[2][1] * Tg[2][1]) + (Kg[2][2] * Tg[2][2])) * (gyroZ_raw - gyro_bias[2]);

    Tio += 0.05;
    networkConsole.printf("Calibrated Accelerometer (X,Y,Z): %f,%f,%f\n", acce_calibrated[0], acce_calibrated[1], acce_calibrated[2]);
    networkConsole.printf("Calibrated Gyroscope (X,Y,Z): %f,%f,%f\n", gyro_calibrated[0], gyro_calibrated[1], gyro_calibrated[2]);



    delay(50);  // Adjust the delay to control the data sending rate
}
