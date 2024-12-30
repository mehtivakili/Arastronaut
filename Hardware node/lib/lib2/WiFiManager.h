#ifndef WIFIMANAGER_H
#define WIFIMANAGER_H

#include <WiFi.h>
#include <WebServer.h>
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
    WebServer& getServer();  // Add this line to provide access to the server instance

private:
    WebServer server;
    void setupAccessPoint();
    void setupWebServer();
    void handleScanNetworks();
    void handleSetWiFi();
    void handleForgetWiFi();
    bool connectToSavedWiFi();
};

#endif
