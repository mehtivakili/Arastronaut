#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>

// Define the custom MAC address
uint8_t customMACAddress[] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x57}; // Example custom MAC address

// Structure to hold incoming data
typedef struct struct_message {
    char text[32];
} struct_message;

struct_message incomingMessage;

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
    memcpy(&incomingMessage, data, sizeof(incomingMessage));
    Serial.print("Received message: ");
    Serial.println(incomingMessage.text);

    // Send acknowledgment back
    esp_now_send(mac_addr, (uint8_t *) &incomingMessage, sizeof(incomingMessage));
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Serial.println("ESP-NOW Receiver");

    // Set the custom MAC address
    esp_err_t result = esp_wifi_set_mac(WIFI_IF_STA, customMACAddress);
    if (result == ESP_OK) {
        Serial.print("Custom MAC Address set to: ");
        Serial.println(WiFi.macAddress());
    } else {
        Serial.print("Failed to set MAC Address, error: ");
        Serial.println(result);
    }

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register receive callback
    esp_now_register_recv_cb(onReceive);
}

void loop() {
    // Do nothing in loop
}
