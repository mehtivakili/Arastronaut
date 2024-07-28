#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "lcdgfx.h"
#include "imu.h"
#include "display.h"
#include "ota.h"
#include "config.h"
#include "globals.h"
#include "link.h"
#include "networking.h" // Include the networking library
#include "uwb.h" // Include UWB library
#include "web_interface.h" // Include Web Interface library

// GLOBALS
DisplaySSD1306_128x64_I2C display(-1);

char shortAddress[6];

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected");
  Serial.print("IP Address:");
  Serial.println(WiFi.localIP());
  delay(500);
}

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__ " " __DATE__);

  setupUWB(); // Initialize UWB

  display.setFixedFont(ssd1306xled_font6x8);
  display.begin();
  display.clear();

#if DEVICE_TYPE == IS_TAG
  display.printFixed(0, 0, "TAG", STYLE_NORMAL);
#else
  display.printFixed(0, 0, "ANCHOR", STYLE_NORMAL);
#endif

  setupWiFi();
  setupWebServer(); // Initialize Web Server

  byte* currentShortAddress = DW1000Ranging.getCurrentShortAddress();
  sprintf(shortAddress, "%02X%02X", currentShortAddress[1], currentShortAddress[0]);
  display.printFixed(0, 8, "02:00:00:00:00:00:02:02", STYLE_NORMAL);
  display.printFixed(0, 16, shortAddress, STYLE_NORMAL);

  Serial.println("Setup complete");

  initIMU();
  setupOTA(); // Initialize OTA updates
  setupNetworking(); // Initialize networking
}

void loop() {
#if DEVICE_TYPE == IS_TAG
  DW1000Ranging.loop();
  handleOTA(); // Handle OTA updates

  if ((millis() - lastUpdateTime) > updateInterval) {
    sendJSON(uwb_data, shortAddress, host, portNum);
    lastUpdateTime = millis();
  }
#endif

  server.handleClient(); // Handle web server
}
