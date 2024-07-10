#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "D-Link";
const char* password = "09124151339";

WiFiServer tcpServer(23);  // Port 23 for Telnet
WiFiClient tcpClient;

void setupOTA() {
  // Port defaults are 3232
  ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("esp32-ota");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
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
  Serial.println("OTA Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start TCP server
  tcpServer.begin();
  tcpServer.setNoDelay(true);
  Serial.println("TCP server started");

  // Setup OTA
  setupOTA();
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();

  // Handle TCP connections
  if (tcpServer.hasClient()) {
    if (!tcpClient || !tcpClient.connected()) {
      if (tcpClient) {
        Serial.println("TCP client disconnected");
        tcpClient.stop();
      }
      tcpClient = tcpServer.available();
      if (tcpClient) {
        Serial.println("New TCP client connected");
        tcpClient.println("Welcome to the ESP32 Serial Monitor!");
        tcpClient.flush();
      }
    }
  }

  // Echo incoming data to Serial Monitor and TCP client
  if (tcpClient && tcpClient.connected()) {
    while (tcpClient.available()) {
      char c = tcpClient.read();
      Serial.write(c); // Echo to Serial Monitor
    }
  }

  if (Serial.available()) {
    char c = Serial.read();
    if (tcpClient && tcpClient.connected()) {
      tcpClient.write(c); // Send Serial input to TCP client
    }
  }
}
