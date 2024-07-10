#include <WiFiManager.h>

WiFiManager wifiManager;

void setup() {
  wifiManager.begin();
}

void loop() {
  wifiManager.handleClient();
}
