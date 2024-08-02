#include <esp_now.h>
#include <WiFi.h>

// Custom MAC Address for the peer ESP32
uint8_t peerAddress[] = {0x24, 0x6F, 0x28, 0xAE, 0x56, 0x1A};

// Structure example to send data
typedef struct struct_message {
    char a[32];
    int b;
    float c;
    bool d;
} struct_message;

struct_message myData;
struct_message incomingData;

esp_now_peer_info_t peerInfo;

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingData, incomingData, sizeof(incomingData));
  struct_message* receivedData = (struct_message*) incomingData;
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(receivedData->a);
  Serial.print("Int: ");
  Serial.println(receivedData->b);
  Serial.print("Float: ");
  Serial.println(receivedData->c);
  Serial.print("Bool: ");
  Serial.println(receivedData->d);
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Set custom MAC address
  uint8_t customMAC[] = {0x24, 0x6F, 0x28, 0xAE, 0x56, 0x1B};
  esp_base_mac_addr_set(customMAC);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register send callback
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Example to send data
  strcpy(myData.a, "World");
  myData.b = random(1, 20);
  myData.c = 3.4;
  myData.d = true;

  esp_err_t result = esp_now_send(peerAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }

  delay(2000);  // Add a delay to avoid congestion
}
