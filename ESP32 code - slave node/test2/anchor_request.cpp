#include <WiFi.h>
#include <WebServer.h>
#include <Arduino.h>

const char* ssid = "ESP32_AP";
const char* password = "123456789";

WebServer server(80);
int blinkFrequency = 1; // Default frequency (1 Hz)
unsigned long previousMillis = 0;
const long interval = 1000; // Default interval for 1 Hz
int ledState = LOW;

void handleModification() {
    if (server.hasArg("setting")) {
        String setting = server.arg("setting");
        blinkFrequency = setting.toInt();
        Serial.println("Received setting: " + setting);
        server.send(200, "text/plain", "Setting applied: " + setting + " Hz");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(2, OUTPUT);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("Connected to WiFi");

    server.on("/modify", handleModification);
    server.begin();
}

void loop() {
    server.handleClient();
    
    unsigned long currentMillis = millis();
    long blinkInterval = 1000 / blinkFrequency; // Calculate interval based on frequency

    if (currentMillis - previousMillis >= blinkInterval) {
        previousMillis = currentMillis;

        // Toggle the LED state
        ledState = !ledState;
        digitalWrite(2, ledState);
    }
}
