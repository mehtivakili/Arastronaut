#include <WiFi.h>
#include <esp_now.h>

// SSID for the AP mode
const char* ssid = "ESP32-AP";
const char* password = "12345678";

// ESP-NOW broadcast address (replace with the actual MAC address of the receiver)
uint8_t broadcastAddress[] = {0x24, 0x0A, 0xC4, 0x12, 0x34, 0x57};

typedef struct struct_message {
  char a[32];
  int b;
  float c;
} struct_message;

struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");
  Serial.println(myData.c);
}

void setup() {
  Serial.begin(115200);

  // Set up AP mode
  WiFi.softAP(ssid, password);
  Serial.print("Setting AP (Access Point)…");
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());

  // Set up ESP-NOW
  WiFi.mode(WIFI_AP_STA); // Ensure both AP and STA modes are active
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo)); // Initialize with zero
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Prepare data to send
  strcpy(myData.a, "Hello ESP-NOW");
  myData.b = 123;
  myData.c = 45.67;
}

void loop() {
  // Send data via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  delay(2000);
}
