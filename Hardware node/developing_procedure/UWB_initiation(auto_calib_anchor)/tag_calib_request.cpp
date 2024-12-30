#include <WiFi.h>
#include <HTTPClient.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

const char* ssid = "ESP32_AP";
const char* password = "123456789";

WebServer server(80);

// Connection pins
const uint8_t PIN_RST = 27; // Reset pin
const uint8_t PIN_IRQ = 34; // IRQ pin
const uint8_t PIN_SS = 5;   // SPI select pin

// TAG antenna delay defaults to 16384
// Leftmost two bytes below will become the "short address"
char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";

// Target distance for calibration
float target_distance = 1.0; // Example target distance in meters

void newRange()
{
  // Simplified to only get the range
  float range = DW1000Ranging.getDistantDevice()->getRange();
  Serial.println(range);
}

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

void handleRoot() {
    String html = "<html><body><h1>Connected Clients</h1><ul>";

    wifi_sta_list_t wifi_sta_list;
    esp_netif_sta_list_t adapter_sta_list;

    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_netif_get_sta_list(&wifi_sta_list, &adapter_sta_list);  // Corrected line

    for (int i = 0; i < adapter_sta_list.num; i++) {
        esp_netif_sta_info_t station = adapter_sta_list.sta[i];
        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", MAC2STR(station.mac));
        html += "<li>MAC: " + String(macStr) + " - IP: " + IPAddress(station.ip.addr).toString() + "</li>";
    }

    html += "</ul>";

    html += "<h2>Start Calibration</h2>";
    html += "<form action=\"/start_calibration\" method=\"POST\">";
    html += "Anchor IP Address: <input type=\"text\" name=\"ip\"><br>";
    html += "Target Distance: <input type=\"text\" name=\"target_distance\"><br>";
    html += "<input type=\"submit\" value=\"Start Calibration\">";
    html += "</form></body></html>";

    server.send(200, "text/html", html);
}

void handleStartCalibration() {
    if (server.hasArg("ip") && server.hasArg("target_distance")) {
        String ip = server.arg("ip");
        String targetDistance = server.arg("target_distance");

        HTTPClient http;
        String url = "http://" + ip + "/start_calibration?target_distance=" + targetDistance + "&tag_ip=" + WiFi.localIP().toString();
        http.begin(url);

        int httpCode = http.GET();
        if (httpCode > 0) {
            String payload = http.getString();
            server.send(200, "text/plain", "Request sent to " + ip + "\nResponse: " + payload);
        } else {
            server.send(500, "text/plain", "Failed to connect to " + ip);
        }
        http.end();
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleReceiveCalibration() {
    if (server.hasArg("final_adelay")) {
        String finalAdelay = server.arg("final_adelay");
        Serial.println("Received final Adelay from anchor: " + finalAdelay);
        // You can store this value or use it as needed
        server.send(200, "text/plain", "Final Adelay received: " + finalAdelay);
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void setup()
{
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/start_calibration", handleStartCalibration);
  server.on("/receive_calibration", handleReceiveCalibration); // Add this line
  server.begin();
  Serial.println("HTTP server started");

  // Init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // Start as tag, do not assign random short address
  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop()
{
  server.handleClient();
  DW1000Ranging.loop();
}
