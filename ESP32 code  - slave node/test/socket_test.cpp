#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <Update.h>
#include <ArduinoOTA.h>

const char* ssid = "D-Link";
const char* password = "09124151339";

WebServer webServer(80);
WebSocketsServer webSocket = WebSocketsServer(81);

void handleWebSocketMessage(uint8_t num, uint8_t *payload, size_t length) {
  payload[length] = 0;
  Serial.printf("WebSocket message received: %s\n", (char*)payload);
  webSocket.sendTXT(num, "Message received: " + String((char*)payload));
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("WebSocket client #%u disconnected\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("WebSocket client #%u connected from %s\n", num, ip.toString().c_str());
        webSocket.sendTXT(num, "Connected");
      }
      break;
    case WStype_TEXT:
      handleWebSocketMessage(num, payload, length);
      break;
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

const char* index_html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 WebSocket</title>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
  <script type="text/javascript">
    var socket;
    function init() {
      socket = io('ws://' + window.location.hostname + ':81');
      socket.on('connect', function() {
        console.log('Connected');
        document.getElementById('status').innerHTML = "Connected";
      });

      socket.on('message', function(data) {
        console.log('Message from server: ', data);
        document.getElementById('messages').innerHTML += '<p>' + data + '</p>';
      });
    }

    function sendMessage() {
      var message = document.getElementById('message').value;
      socket.send(message);
    }
  </script>
</head>
<body onload="init()">
  <h1>ESP32 WebSocket</h1>
  <p id="status">Connecting...</p>
  <input type="text" id="message" placeholder="Enter message">
  <button onclick="sendMessage()">Send</button>
  <div id="messages"></div>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");

  ArduinoOTA.setHostname("esp32");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }
    Serial.println("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });

  ArduinoOTA.begin();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);

  webServer.on("/", HTTP_GET, []() {
    webServer.send(200, "text/html", index_html);
  });

  webServer.on("/update", HTTP_POST, []() {
    webServer.sendHeader("Connection", "close");
    webServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = webServer.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.setDebugOutput(true);
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
      Serial.setDebugOutput(false);
    }
    yield();
  });

  webServer.begin();
}

void loop() {
  ArduinoOTA.handle();
  webServer.handleClient();
  webSocket.loop();
}
