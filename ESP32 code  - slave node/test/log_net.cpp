#include <NetworkConsole.h>
#include "WiFiManager.h" // Include your custom WiFiManager

NetworkConsole networkConsole;
WiFiManager wifiManager;

void setup() {
    Serial.begin(115200);

    // Start WiFiManager and connect to WiFi
    wifiManager.begin();

    // Start the network console
    networkConsole.begin();
    networkConsole.println("ESP32 Network Console started.");
}

void loop() {
    wifiManager.handleClient(); // Ensure WiFiManager handles client requests
    networkConsole.print("Logging a message over the network...");
    networkConsole.println(" This is a new line.");
    delay(2000);

    ArduinoOTA.handle(); // Handle OTA updates
}
