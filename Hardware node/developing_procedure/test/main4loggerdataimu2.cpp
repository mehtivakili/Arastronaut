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

float acce_calibrated[3];
float gyro_calibrated[3];

double Tio = 0.0;

float Ta[3][3] = {{1, -0.00546066, 0.00101399}, {0, 1, 0.00141895}, {0, 0, 1}};
float Ka[3][3] = {{0.00358347, 0, 0}, {0, 0.00358133, 0}, {0, 0, 0.00359205}};
float Tg[3][3] = {{1, -0.00614889, -0.000546488}, {0.0102258, 1, 0.000838491}, {0.00412113, 0.0020154, 1}};
float Kg[3][3] = {{0.000531972, 0, 0}, {0, 0.000531541, 0}, {0, 0, 0.000531}};
float acce_bias[3] = {-8.28051, -4.6756, -0.870355};
float gyro_bias[3] = {4.53855, 4.001, -1.9779};

// HTML page for configuration
const char* plotPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 IMU and Gyro Plotter</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: Arial, sans-serif; display: flex; }
    .config-section { width: 40%; padding: 20px; }
    .plot-section { width: 60%; padding: 20px; }
    .data-table { margin-top: 20px; }
    table { width: 100%; border-collapse: collapse; }
    th, td { padding: 10px; border: 1px solid #ddd; text-align: left; }
  </style>
</head>
<body>
  <div class="config-section">
    <h1>IMU Configuration</h1>
    <form id="configForm">
      <h2>Ta Matrix</h2>
      <div id="TaMatrix">
        <!-- Ta matrix inputs will be dynamically generated here -->
      </div>
      <h2>Ka Matrix</h2>
      <div id="KaMatrix">
        <!-- Ka matrix inputs will be dynamically generated here -->
      </div>
      <h2>Tg Matrix</h2>
      <div id="TgMatrix">
        <!-- Tg matrix inputs will be dynamically generated here -->
      </div>
      <h2>Kg Matrix</h2>
      <div id="KgMatrix">
        <!-- Kg matrix inputs will be dynamically generated here -->
      </div>
      <h2>Accelerometer Bias</h2>
      <div id="accBias">
        <!-- Accelerometer bias inputs will be dynamically generated here -->
      </div>
      <h2>Gyroscope Bias</h2>
      <div id="gyroBias">
        <!-- Gyroscope bias inputs will be dynamically generated here -->
      </div>
      <button type="button" onclick="updateConfig()">Update Config</button>
      <button type="button" onclick="resetConfig()">Reset to Default</button>
    </form>
  </div>
  <div class="plot-section">
    <h1>ESP32 IMU and Gyroscope Real-Time Plotter</h1>
    <table class="data-table" id="dataTable">
      <thead>
        <tr>
          <th>Time</th>
          <th>Accel X</th>
          <th>Accel Y</th>
          <th>Accel Z</th>
          <th>Gyro X</th>
          <th>Gyro Y</th>
          <th>Gyro Z</th>
        </tr>
      </thead>
      <tbody id="dataBody">
        <!-- Data rows will be dynamically generated here -->
      </tbody>
    </table>
  </div>
  <script>
    let ws;
    const defaultConfig = {
      Ta: [[1, -0.00546066, 0.00101399], [0, 1, 0.00141895], [0, 0, 1]],
      Ka: [[0.00358347, 0, 0], [0, 0.00358133, 0], [0, 0, 0.00359205]],
      Tg: [[1, -0.00614889, -0.000546488], [0.0102258, 1, 0.000838491], [0.00412113, 0.0020154, 1]],
      Kg: [[0.000531972, 0, 0], [0, 0.000531541, 0], [0, 0, 0.000531]],
      accBias: [-8.28051, -4.6756, -0.870355],
      gyroBias: [4.53855, 4.001, -1.9779]
    };

    document.addEventListener("DOMContentLoaded", function() {
      initializeForm('TaMatrix', 'Ta', 3, 3);
      initializeForm('KaMatrix', 'Ka', 3, 3);
      initializeForm('TgMatrix', 'Tg', 3, 3);
      initializeForm('KgMatrix', 'Kg', 3, 3);
      initializeForm('accBias', 'accBias', 3, 1);
      initializeForm('gyroBias', 'gyroBias', 3, 1);
      resetConfig();
      initializeWebSocket();
    });

    function initializeForm(containerId, name, rows, cols) {
      const container = document.getElementById(containerId);
      for (let i = 0; i < rows; i++) {
        for (let j = 0; j < cols; j++) {
          const input = document.createElement('input');
          input.type = 'number';
          input.step = '0.001';
          input.name = `${name}[${i}][${j}]`;
          input.style.width = '50px';
          container.appendChild(input);
        }
        container.appendChild(document.createElement('br'));
      }
    }

    function updateConfig() {
      const formData = new FormData(document.getElementById('configForm'));
      const config = {
        Ta: [[formData.get('Ta[0][0]'), formData.get('Ta[0][1]'), formData.get('Ta[0][2]')],
             [formData.get('Ta[1][0]'), formData.get('Ta[1][1]'), formData.get('Ta[1][2]')],
             [formData.get('Ta[2][0]'), formData.get('Ta[2][1]'), formData.get('Ta[2][2]')]],
        Ka: [[formData.get('Ka[0][0]'), formData.get('Ka[0][1]'), formData.get('Ka[0][2]')],
             [formData.get('Ka[1][0]'), formData.get('Ka[1][1]'), formData.get('Ka[1][2]')],
             [formData.get('Ka[2][0]'), formData.get('Ka[2][1]'), formData.get('Ka[2][2]')]],
        Tg: [[formData.get('Tg[0][0]'), formData.get('Tg[0][1]'), formData.get('Tg[0][2]')],
             [formData.get('Tg[1][0]'), formData.get('Tg[1][1]'), formData.get('Tg[1][2]')],
             [formData.get('Tg[2][0]'), formData.get('Tg[2][1]'), formData.get('Tg[2][2]')]],
        Kg: [[formData.get('Kg[0][0]'), formData.get('Kg[0][1]'), formData.get('Kg[0][2]')],
             [formData.get('Kg[1][0]'), formData.get('Kg[1][1]'), formData.get('Kg[1][2]')],
             [formData.get('Kg[2][0]'), formData.get('Kg[2][1]'), formData.get('Kg[2][2]')]],
        accBias: [formData.get('accBias[0][0]'), formData.get('accBias[1][0]'), formData.get('accBias[2][0]')],
        gyroBias: [formData.get('gyroBias[0][0]'), formData.get('gyroBias[1][0]'), formData.get('gyroBias[2][0]')]
      };
      fetch('/updateConfig', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(config)
      }).then(response => {
        if (response.ok) {
          alert('Configuration updated successfully');
        } else {
          alert('Failed to update configuration');
        }
      }).catch(error => {
        console.error('Error updating configuration:', error);
      });
    }

    function resetConfig() {
      setConfigForm(defaultConfig);
    }

    function setConfigForm(config) {
      setMatrixValues('Ta', config.Ta);
      setMatrixValues('Ka', config.Ka);
      setMatrixValues('Tg', config.Tg);
      setMatrixValues('Kg', config.Kg);
      setMatrixValues('accBias', config.accBias);
      setMatrixValues('gyroBias', config.gyroBias);
    }

    function setMatrixValues(name, matrix) {
      for (let i = 0; i < matrix.length; i++) {
        for (let j = 0; j < matrix[i].length; j++) {
          document.querySelector(`input[name="${name}[${i}][${j}]"]`).value = matrix[i][j];
        }
      }
    }

    function initializeWebSocket() {
      console.log("Initializing WebSocket connection...");
      ws = new WebSocket(`ws://${location.hostname}:81/ws`);

      ws.onopen = function() {
        console.log("WebSocket connection established");
      };

      ws.onmessage = function(event) {
        console.log("Received data:", event.data); // Debug log for incoming data
        try {
          const data = JSON.parse(event.data);
          const time = data.time.toFixed(3);
          const accelX = data.accel[0].toFixed(3);
          const accelY = data.accel[1].toFixed(3);
          const accelZ = data.accel[2].toFixed(3);
          const gyroX = data.gyro[0].toFixed(3);
          const gyroY = data.gyro[1].toFixed(3);
          const gyroZ = data.gyro[2].toFixed(3);

          const newRow = document.createElement('tr');
          newRow.innerHTML = `<td>${time}</td><td>${accelX}</td><td>${accelY}</td><td>${accelZ}</td><td>${gyroX}</td><td>${gyroY}</td><td>${gyroZ}</td>`;

          const dataBody = document.getElementById('dataBody');
          dataBody.insertBefore(newRow, dataBody.firstChild);

          if (dataBody.rows.length > 20) {
            dataBody.deleteRow(20);
          }
        } catch (e) {
          console.error("Error parsing JSON:", e);
        }
      };

      ws.onerror = function(error) {
        console.log("WebSocket error:", error); // Debug log for WebSocket errors
      };

      ws.onclose = function() {
        console.log("WebSocket connection closed");
      };
    }
  </script>
</body>
</html>
)rawliteral";

String formatFloat(float value) {
  return String(value, 3);
}
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
  } else if(type == WStype_PONG) {
    webSocket.sendPing(num);
  } else {
    Serial.printf("[%u] Unknown WebSocket event\n", num);
  }
}

void sendIMUData() {
  StaticJsonDocument<256> doc;
  doc["time"] = Tio;
  JsonArray accel = doc.createNestedArray("accel");
  accel.add(formatFloat(acce_calibrated[0]).toFloat());
  accel.add(formatFloat(acce_calibrated[1]).toFloat());
  accel.add(formatFloat(acce_calibrated[2]).toFloat());

  JsonArray gyro = doc.createNestedArray("gyro");
  gyro.add(formatFloat(gyro_calibrated[0]).toFloat());
  gyro.add(formatFloat(gyro_calibrated[1]).toFloat());
  gyro.add(formatFloat(gyro_calibrated[2]).toFloat());


  String output;
  serializeJson(doc, output);

bool result = webSocket.broadcastTXT(output);
if (result) {
  Serial.println("Data sent successfully");
} else {
  Serial.println("Failed to send data");
}
}

void handleUpdateConfig() {
  if (wifiManager.getServer().hasArg("plain") == false) { 
    wifiManager.getServer().send(400, "text/plain", "Body not received");
    return;
  }
  String body = wifiManager.getServer().arg("plain");
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, body);

  if (error) {
    wifiManager.getServer().send(400, "text/plain", "Invalid JSON");
    return;
  }

  JsonArray TaArray = doc["Ta"].as<JsonArray>();
  JsonArray KaArray = doc["Ka"].as<JsonArray>();
  JsonArray TgArray = doc["Tg"].as<JsonArray>();
  JsonArray KgArray = doc["Kg"].as<JsonArray>();
  JsonArray accBiasArray = doc["accBias"].as<JsonArray>();
  JsonArray gyroBiasArray = doc["gyroBias"].as<JsonArray>();

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Ta[i][j] = TaArray[i][j];
      Ka[i][j] = KaArray[i][j];
      Tg[i][j] = TgArray[i][j];
      Kg[i][j] = KgArray[i][j];
    }
    acce_bias[i] = accBiasArray[i];
    gyro_bias[i] = gyroBiasArray[i];
  }

  wifiManager.getServer().send(200, "text/plain", "Configuration updated");
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
  wifiManager.getServer().on("/updateConfig", HTTP_POST, handleUpdateConfig);

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
