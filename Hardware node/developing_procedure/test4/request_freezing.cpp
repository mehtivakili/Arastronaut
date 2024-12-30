#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32_AP";
const char* password = "123456789";

WebServer server(80);

// Mode definitions
#define IMU_ONLY 1
#define UWB_CALIBRATION 2
#define UWB_DATA 3
#define IMU_UWB_DATA 4

int currentMode = 0;
bool calibration_in_progress = false;

void handleModeChange() {
  if (server.hasArg("mode")) {
    int mode = server.arg("mode").toInt();
    Serial.print("Requested mode: ");
    Serial.println(mode);
    switch (mode) {
      case IMU_ONLY:
        currentMode = IMU_ONLY;
        server.send(200, "text/plain", "Mode set to IMU_ONLY");
        break;
      case UWB_CALIBRATION:
        currentMode = UWB_CALIBRATION;
        calibration_in_progress = true;
        server.send(200, "text/plain", "Mode set to UWB_CALIBRATION");
        break;
      case UWB_DATA:
        currentMode = UWB_DATA;
        server.send(200, "text/plain", "Mode set to UWB_DATA");
        break;
      case IMU_UWB_DATA:
        currentMode = IMU_UWB_DATA;
        server.send(200, "text/plain", "Mode set to IMU_UWB_DATA");
        break;
      default:
        server.send(400, "text/plain", "Invalid mode");
        break;
    }
  } else {
    server.send(400, "text/plain", "Mode not specified");
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);

  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/mode", handleModeChange);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
}
