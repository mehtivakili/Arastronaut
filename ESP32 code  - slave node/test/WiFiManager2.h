#ifndef WIFIMANAGER_H
#define WIFIMANAGER_H

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>

// Network credentials for Access Point mode
extern const char* ap_ssid;
extern const char* ap_password;

// Preferences for storing Wi-Fi credentials
extern Preferences preferences;

class WiFiManager {
public:
    WiFiManager();
    void begin();
    void handleClient();
    AsyncWebServer& getServer();

private:
    AsyncWebServer server;
    void setupAccessPoint();
    void setupWebServer();
    void handleScanNetworks(AsyncWebServerRequest *request);
    void handleSetWiFi(AsyncWebServerRequest *request);
    void handleForgetWiFi(AsyncWebServerRequest *request);
    bool connectToSavedWiFi();
};

#endif
