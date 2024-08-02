#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// List of MAC addresses of the receivers
uint8_t receiverMACAddresses[][6] = {
    {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x56},
        {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x57},
            {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x58},
                {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x59}
    // Add other receiver MAC addresses here
};

// Structure to hold outgoing data
typedef struct struct_message {
    char text[32];
} struct_message;

struct_message message;
bool responses[sizeof(receiverMACAddresses) / 6];

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Message Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");

    // Update response status based on MAC address
    for (int i = 0; i < sizeof(receiverMACAddresses) / 6; i++) {
        if (memcmp(mac_addr, receiverMACAddresses[i], 6) == 0) {
            responses[i] = (status == ESP_NOW_SEND_SUCCESS);
            break;
        }
    }
}

void onReceive(const uint8_t *mac_addr, const uint8_t *data, int len) {
    // This callback can be used for additional handling if needed
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    Serial.println("ESP-NOW Sender");

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register send and receive callbacks
    esp_now_register_send_cb(onSent);
    esp_now_register_recv_cb(onReceive);

    // Add all receivers as peers
    for (int i = 0; i < sizeof(receiverMACAddresses) / 6; i++) {
        esp_now_peer_info_t peerInfo = {};
        memcpy(peerInfo.peer_addr, receiverMACAddresses[i], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Failed to add peer");
        }
    }

    // Set the message
    strcpy(message.text, "Discovery Message");
}

void loop() {
    // Reset responses
    memset(responses, 0, sizeof(responses));

    // Send discovery message to all receivers
    for (int i = 0; i < sizeof(receiverMACAddresses) / 6; i++) {
        esp_err_t result = esp_now_send(receiverMACAddresses[i], (uint8_t *) &message, sizeof(message));
        if (result != ESP_OK) {
            Serial.print("Error sending message to: ");
            for (int j = 0; j < 6; j++) {
                Serial.print(receiverMACAddresses[i][j], HEX);
                if (j < 5) Serial.print(":");
            }
            Serial.println();
        }
    }

    // Wait for responses
    delay(2000);

    // Print available devices
    Serial.println("Available devices:");
    for (int i = 0; i < sizeof(receiverMACAddresses) / 6; i++) {
        if (responses[i]) {
            Serial.print("Device with MAC: ");
            for (int j = 0; j < 6; j++) {
                Serial.print(receiverMACAddresses[i][j], HEX);
                if (j < 5) Serial.print(":");
            }
            Serial.println();
        }
    }

    // Delay before next discovery
    delay(10000); // Discover every 10 seconds
}
