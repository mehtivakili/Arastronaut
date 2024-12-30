
 #include <WiFiManager.h>
#include <NetworkConsole.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include "BMI088.h"
#include "config2.h"
NetworkConsole networkConsole;
WiFiManager wifiManager;

WebSocketsServer webSocket = WebSocketsServer(81);

Bmi088Accel accel(SPI, 32);
Bmi088Gyro gyro(SPI, 25);

int16_t accelX_raw, accelY_raw, accelZ_raw;
int16_t gyroX_raw, gyroY_raw, gyroZ_raw;
// float Ta[3][3] = {{1, -0.00546066, 0.00101399}, {0, 1, 0.00141895}, {0, 0, 1}};
// float Ka[3][3] = {{0.00358347, 0, 0}, {0, 0.00358133, 0}, {0, 0, 0.00359205}};
// float acce_bias[3] = {-8.28051, -4.6756, -0.870355};
// float Tg[3][3] = {{1, -0.00614889, -0.000546488}, {0.0102258, 1, 0.000838491}, {0.00412113, 0.0020154, 1}};
// float Kg[3][3] = {{0.000531972, 0, 0}, {0, 0.000531541, 0}, {0, 0, 0.000531}};
// float gyro_bias[3] = {4.53855, 4.001, -1.9779};

float acce_calibrated[3];
float gyro_calibrated[3];

double Tio = 0.0;

// HTML page for configuration
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
  const accelCtx = document.getElementById('accelChart').getContext('2d');
  const gyroCtx = document.getElementById('gyroChart').getContext('2d');

  const accelChart = new Chart(accelCtx, {
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

  const gyroChart = new Chart(gyroCtx, {
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

const ws = new WebSocket(`ws://192.168.2.3:81/ws`);

ws.onopen = function() {
    console.log("WebSocket connection established");
};

ws.onmessage = function(event) {
    console.log("Received data:", event.data); // Debug log for incoming data
    const data = JSON.parse(event.data);
    const time = data.time;

    console.log("Parsed data:", data); // Debug log for parsed data

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
};

ws.onerror = function(error) {
    console.log("WebSocket error:", error); // Debug log for WebSocket errors
};

ws.onclose = function(event) {
    console.log("WebSocket connection closed", event.code, event.reason);
    setTimeout(function() {
        console.log("Reconnecting...");
        ws = new WebSocket(`ws://192.168.2.3:81/ws`);
    }, 1000); // Try to reconnect after 1 second
};

</script>

</body>
</html>
)rawliteral";

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_DISCONNECTED) {
        Serial.printf("[%u] Disconnected!\n", num);
    } else if (type == WStype_CONNECTED) {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %s\n", num, ip.toString().c_str());
    } else if (type == WStype_TEXT) {
        Serial.printf("[%u] Received Text: %s\n", num, payload);
    } else if (type == WStype_BIN) {
        Serial.printf("[%u] Received Binary\n", num);
    } else {
        Serial.print(type);
        Serial.printf("[%u] Unknown WebSocket event\n", num);
    }
}



void sendIMUData() {
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

  // Serial.println("Sending IMU Data: ");
  // Serial.println(output);

  // bool result = webSocket.broadcastTXT(output);
  // if (result) {
  //   Serial.println("Data sent successfully");
  // } else {
  //   Serial.println("Failed to send data");
  // }
}


void setup() {
  Serial.begin(115200);

  wifiManager.begin();
  networkConsole.begin();
  networkConsole.println("ESP32 Network Console started.");

  if (accel.begin(Bmi088Accel::RANGE_12G, Bmi088Accel::ODR_1600HZ_BW_234HZ) < 0 ||
      gyro.begin(Bmi088Gyro::RANGE_1000DPS, Bmi088Gyro::ODR_2000HZ_BW_230HZ) < 0) {
    Serial.println("Sensor initialization failed");
    while (1);
  }

  accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
  gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Serve the plot page
  wifiManager.getServer().on("/", []() {
    wifiManager.getServer().send(200, "text/html", plotPage);
  });
  wifiManager.getServer().on("/plot", []() {
  wifiManager.getServer().send(200, "text/html", plotPage);
});

  wifiManager.getServer().begin();
  Serial.println("HTTP server started");
}

void loop() {
  wifiManager.handleClient();
  webSocket.loop();
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

  sendIMUData();
  delay(50);  // Adjust the delay to control the data sending rate
}


