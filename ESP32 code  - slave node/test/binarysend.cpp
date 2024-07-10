
// HTML page for configuration
const char* plotPage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 IMU and Gyro Plotter</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>

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
      <button type="button" onclick="downloadData()">Download Data</button>
      <button type="button" onclick="resetTio()">Reset Time</button>

    <h2>Data Collection</h2>
    <label for="collectionDuration">Collection Duration (s):</label>
    <input type="number" id="collectionDuration" name="collectionDuration" min="1" step="1" value="10">
    <br>
    <label for="collectionInterval">Collection Interval (ms):</label>
    <input type="number" id="collectionInterval" name="collectionInterval" min="50" step="10" value="20">
    <br>
    <button type="button" onclick="startDataCollection()">Start Data Collection</button>
    <button type="button" onclick="stopDataCollection()">Stop Data Collection</button>
    <p id="elapsedTime">Elapsed Time: 0s</p>
    <label for="intervalDuration">Interval Duration (ms):</label>
<input type="number" id="intervalDuration" name="intervalDuration" min="1" value="100">


    </form>
  </div>
  <div class="plot-section">
    <h1>ESP32 IMU and Gyroscope Real-Time Plotter</h1>
      <div id="accelChart" style="width:100%; height:400px;"></div>
      <div id="gyroChart" style="width:100%; height:400px;"></div>
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
  let time = 0;
  let collectedData = [];
  let collectingData = false;
  let startTime = null;
  let collectionTimer;
  let elapsedTimeInterval;
  let intervalDuration = parseInt(document.getElementById('intervalDuration').value);
const accelChartCtx = document.getElementById('accelChart').getContext('2d');
const gyroChartCtx = document.getElementById('gyroChart').getContext('2d');

const accelChart = new Chart(accelChartCtx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            { label: 'Accel X', data: [], borderColor: 'red', fill: false },
            { label: 'Accel Y', data: [], borderColor: 'green', fill: false },
            { label: 'Accel Z', data: [], borderColor: 'blue', fill: false }
        ]
    },
    options: {
        scales: {
            x: { type: 'linear', position: 'bottom' }
        }
    }
});

const gyroChart = new Chart(gyroChartCtx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [
            { label: 'Gyro X', data: [], borderColor: 'red', fill: false },
            { label: 'Gyro Y', data: [], borderColor: 'green', fill: false },
            { label: 'Gyro Z', data: [], borderColor: 'blue', fill: false }
        ]
    },
    options: {
        scales: {
            x: { type: 'linear', position: 'bottom' }
        }
    }
});
document.getElementById('intervalDuration').addEventListener('change', function() {
    intervalDuration = parseInt(this.value);
});
  let formattedTime = 0;
  let elapsedTime = 0;
  let manualTime = 0;
  let collectionDuration = 10000; // Default 10 seconds
  let collectionInterval = 20; // Default 100 milliseconds
  // Global variables for plot data
        let accPlotData = {
        x: [],
        y: [],
        z: [],
        time: []
        };
        let gyroPlotData = {
        x: [],
        y: [],
        z: [],
        time: []
        };
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
        initializePlots();      // setInterval(collectData, collectionInterval);
      // setTimeout(stopCollectingData, collectionDuration);
    });
// Initialize plots
function initPlots() {
  Plotly.newPlot('acceleration-plot', [
    { x: [], y: [], mode: 'lines', name: 'Acc X' },
    { x: [], y: [], mode: 'lines', name: 'Acc Y' },
    { x: [], y: [], mode: 'lines', name: 'Acc Z' }
  ], { title: 'Acceleration Data', xaxis: { title: 'Time (s)' }, yaxis: { title: 'Acceleration' } });

  Plotly.newPlot('gyroscope-plot', [
    { x: [], y: [], mode: 'lines', name: 'Gyro X' },
    { x: [], y: [], mode: 'lines', name: 'Gyro Y' },
    { x: [], y: [], mode: 'lines', name: 'Gyro Z' }
  ], { title: 'Gyroscope Data', xaxis: { title: 'Time (s)' }, yaxis: { title: 'Gyroscope' } });
}


// Update plots with new data
const MAX_SAMPLES = 200;


    function updatePlots(data) {
        const accelTime = data.time;
        const accelX = data.accel[0];
        const accelY = data.accel[1];
        const accelZ = data.accel[2];
        const gyroX = data.gyro[0];
        const gyroY = data.gyro[1];
        const gyroZ = data.gyro[2];

        Plotly.extendTraces('accelChart', {
            x: [[accelTime], [accelTime], [accelTime]],
            y: [[accelX], [accelY], [accelZ]]
        }, [0, 1, 2]);

        Plotly.extendTraces('gyroChart', {
            x: [[accelTime], [accelTime], [accelTime]],
            y: [[gyroX], [gyroY], [gyroZ]]
        }, [0, 1, 2]);

        const MAX_POINTS = 200;
        const accelChart = document.getElementById('accelChart');
        const gyroChart = document.getElementById('gyroChart');

        if (accelChart.data[0].x.length > MAX_POINTS) {
            Plotly.relayout('accelChart', {
                xaxis: {
                    range: [
                        Math.min(...accelChart.data[0].x) + 1,
                        Math.max(...accelChart.data[0].x)
                    ]
                }
            });
        }

        if (gyroChart.data[0].x.length > MAX_POINTS) {
            Plotly.relayout('gyroChart', {
                xaxis: {
                    range: [
                        Math.min(...gyroChart.data[0].x) + 1,
                        Math.max(...gyroChart.data[0].x)
                    ]
                }
            });
        }
    }


    function initializePlots() {
        Plotly.newPlot('accelChart', [
            { x: [], y: [], mode: 'lines', name: 'Accel X' },
            { x: [], y: [], mode: 'lines', name: 'Accel Y' },
            { x: [], y: [], mode: 'lines', name: 'Accel Z' }
        ], {
            title: 'Accelerometer Data',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Acceleration (m/s²)' }
        });

        Plotly.newPlot('gyroChart', [
            { x: [], y: [], mode: 'lines', name: 'Gyro X' },
            { x: [], y: [], mode: 'lines', name: 'Gyro Y' },
            { x: [], y: [], mode: 'lines', name: 'Gyro Z' }
        ], {
            title: 'Gyroscope Data',
            xaxis: { title: 'Time (s)' },
            yaxis: { title: 'Angular Velocity (°/s)' }
        });
    }

    function resetTio() {
    // fetch('/resetTio', {
    //     method: 'POST'
    // }).then(response => {
    //     if (response.ok) {
    //     //alert('Tio has been reset');
    //     } else {
    //     alert('Failed to reset Tio');
    //     }
    // }).catch(error => {
    //     console.error('Error resetting Tio:', error);
    // });
    // resetTime = formattedTime;
    manualTime = 0;
    console.log(formattedTime);
    
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
    function collectData() {
        if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ action: 'collectData' }));
        }
    }


 function startDataCollection() {
        const durationElement = document.getElementById('collectionDuration');
        const intervalElement = document.getElementById('collectionInterval');

        if (durationElement && intervalElement) {
            const collectionDuration = parseInt(durationElement.value) * 1000;
            const collectionInterval = parseInt(intervalElement.value);
            let elapsedTime = 0;
            document.getElementById('elapsedTime').innerText = `Elapsed Time: ${elapsedTime}s`;
            collectedData = [];
            startTime = new Date().getTime();

            clearInterval(elapsedTimeInterval); // Clear any existing intervals
            clearTimeout(collectionTimer); // Clear any existing timeouts

            elapsedTimeInterval = setInterval(() => {
                elapsedTime += 1;
                document.getElementById('elapsedTime').innerText = `Elapsed Time: ${elapsedTime}s`;
            }, 1000);

            collectionTimer = setTimeout(stopDataCollection, collectionDuration);
        } else {
            console.error('Collection duration or interval input elements not found');
        }
    }

    function stopDataCollection() {
        clearInterval(elapsedTimeInterval);
        clearTimeout(collectionTimer);
        console.log('Data collection stopped.');
    }

    document.addEventListener('DOMContentLoaded', function() {
        initializePlots();
        document.getElementById('startCollectionButton').addEventListener('click', startDataCollection);
        document.getElementById('stopCollectionButton').addEventListener('click', stopDataCollection);
    });


function stopDataCollection() {
  clearInterval(elapsedTimeInterval);
  clearTimeout(collectionTimer);
  collectingData = false;
  console.log('Data collection stopped.');
}
function downloadData() {
  const accData = collectedData.map(row => 
    `${parseFloat(row[0]).toExponential(7)}   ${parseFloat(row[1]).toExponential(7)}   ${parseFloat(row[2]).toExponential(7)}   ${parseFloat(row[3]).toExponential(7)}`).join('\n');
  const gyroData = collectedData.map(row => 
    `${parseFloat(row[0]).toExponential(7)}   ${parseFloat(row[4]).toExponential(7)}   ${parseFloat(row[5]).toExponential(7)}   ${parseFloat(row[6]).toExponential(7)}`).join('\n');
  
  const accCsvContent = "data:text/csv;charset=utf-8," + accData;
  const gyroCsvContent = "data:text/csv;charset=utf-8," + gyroData;

  const accEncodedUri = encodeURI(accCsvContent);
  const gyroEncodedUri = encodeURI(gyroCsvContent);

  const accLink = document.createElement("a");
  accLink.setAttribute("href", accEncodedUri);
  accLink.setAttribute("download", "acc.csv");
  document.body.appendChild(accLink); // Required for FF

  const gyroLink = document.createElement("a");
  gyroLink.setAttribute("href", gyroEncodedUri);
  gyroLink.setAttribute("download", "gyro.csv");
  document.body.appendChild(gyroLink); // Required for FF

  accLink.click();
  gyroLink.click();

  document.body.removeChild(accLink);
  document.body.removeChild(gyroLink);
}

    function initializeWebSocket() {
      console.log("Initializing WebSocket connection...");
      ws = new WebSocket(`ws://${location.hostname}:81/ws`);

      ws.onopen = function() {
        console.log("WebSocket connection established");
      };

ws.onmessage = function(event) {
    const reader = new FileReader();
    reader.onload = function() {
        const arrayBuffer = reader.result;
        const dataView = new DataView(arrayBuffer);
        
        // Decode the time (float64)
        // const time = dataView.getFloat64(0, true);
        
        // // Decode accel data (int16)
        // const accelX = dataView.getInt16(8, true);
        // const accelY = dataView.getInt16(10, true);
        // const accelZ = dataView.getInt16(12, true);
        
        // // Decode gyro data (int16)
        // const gyroX = dataView.getInt16(14, true);
        // const gyroY = dataView.getInt16(16, true);
        // const gyroZ = dataView.getInt16(18, true);
        const acc = [];
        for (let i = 0; i < 3; i++) {
            acc.push(dataView.getFloat32(8 + i * 4, true)); // Assuming little-endian
        }

        // Decode the float array (3 floats, each 4 bytes)
        const gyro = [];
        for (let i = 0; i < 3; i++) {
            gyro.push(dataView.getFloat32(20 + i * 4, true)); // Assuming little-endian
        }
               // Get the current time
        const currentTime = new Date().getTime();

        // Set the start time if it's not already set
        if (!startTime) {
            startTime = currentTime;
        }

        // Calculate the elapsed time in seconds
        const elapsedTime = (currentTime - startTime) / 1000;

        formattedTime = elapsedTime.toFixed(3);
        const newRow = document.createElement('tr');
        newRow.innerHTML = `<td>${formattedTime}</td><td>${acc[0]}</td><td>${acc[1]}</td><td>${acc[2]}</td><td>${gyro[0]}</td><td>${gyro[1]}</td><td>${gyro[2]}</td>`;

        const dataBody = document.getElementById('dataBody');
        dataBody.insertBefore(newRow, dataBody.firstChild);

        // Collect data for download
        collectedData.push([formattedTime, acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]]);

        if (dataBody.rows.length > 20) {
            dataBody.deleteRow(20);
        }

        const data = {
            time: formattedTime,
            accel: [acc[0], acc[1], acc[2]],
            gyro: [gyro[0], gyro[1], gyro[2]]
        };

        updatePlots(data);
    };
    reader.readAsArrayBuffer(event.data);
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

void handleResetTio() {
  Tio = 0.0;
      wifiManager.getServer().send(200, "text/plain", "Tio has been reset");

  // server.send(200, "text/plain", "Tio has been reset");
}
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
  uint8_t buffer[32]; // 8 bytes for double + 3*4 bytes for each float array (accel and gyro)

  // Ensure data fits in the buffer
  static_assert(sizeof(Tio) == 8, "Size of double is not 8 bytes");
  static_assert(sizeof(acce_calibrated) == 12, "Size of accel data is not 12 bytes");
  static_assert(sizeof(gyro_calibrated) == 12, "Size of gyro data is not 12 bytes");

  // Pack data into buffer
  memcpy(buffer, &Tio, sizeof(Tio));
  memcpy(buffer + 8, acce_calibrated, sizeof(acce_calibrated));
  memcpy(buffer + 20, gyro_calibrated, sizeof(gyro_calibrated));

  // Decode the data for verification
  double decodedTio;
  float decodedAccel[3];
  float decodedGyro[3];

  memcpy(&decodedTio, buffer, sizeof(decodedTio));
  memcpy(decodedAccel, buffer + 8, sizeof(decodedAccel));
  memcpy(decodedGyro, buffer + 20, sizeof(decodedGyro));

  // Send the data
  bool result = webSocket.broadcastBIN(buffer, sizeof(buffer));
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

    // Add a handler for the reset Tio request
  wifiManager.getServer().on("/resetTio", []() {
    handleResetTio();
  });

  Serial.println("HTTP server started");
  wifiManager.getServer().on("/resetTio", HTTP_POST, []() {
  Tio = 0.0;
  wifiManager.getServer().send(200, "text/plain", "Tio reset successfully");
});
}

void loop() {
  wifiManager.handleClient();
  webSocket.loop();
  ArduinoOTA.handle(); // Handle OTA updates

  accel.readSensor();
  gyro.readSensor();

  accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
  gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

  // acce_calibrated[0] = ((Ka[0][0] * Ta[0][0]) + (Ka[0][1] * Ta[1][1]) + (Ka[0][2] * Ta[2][2])) * (accelX_raw - acce_bias[0]);
  // acce_calibrated[1] = ((Ka[1][1] * Ta[1][1]) + (Ka[1][2] * Ta[2][2])) * (accelY_raw - acce_bias[1]);
  // acce_calibrated[2] = ((Ka[2][2] * Ta[2][2])) * (accelZ_raw - acce_bias[2]);
  // gyro_calibrated[0] = ((Kg[0][0] * Tg[0][0]) + (Kg[0][1] * Tg[1][1]) + (Kg[0][2] * Tg[2][2])) * (gyroX_raw - gyro_bias[0]);
  // gyro_calibrated[1] = ((Kg[1][0] * Tg[1][0]) + (Kg[1][1] * Tg[1][1]) + (Kg[1][2] * Tg[2][2])) * (gyroY_raw - gyro_bias[1]);
  // gyro_calibrated[2] = ((Kg[2][0] * Tg[2][0]) + (Kg[2][1] * Tg[2][1]) + (Kg[2][2] * Tg[2][2])) * (gyroZ_raw - gyro_bias[2]);

  acce_calibrated[0] = accelX_raw;
  acce_calibrated[1] = accelY_raw;
  acce_calibrated[2] = accelZ_raw;
  gyro_calibrated[0] = gyroX_raw;
  gyro_calibrated[1] = gyroY_raw;
  gyro_calibrated[2] = gyroZ_raw;

  Tio += 0.005;

  sendIMUData();
  delay(5);  // Adjust the delay to control the data sending rate
}
