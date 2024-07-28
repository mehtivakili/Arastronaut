#include "networking.h" // Include the networking header file
#include <ArduinoJson.h> // Include ArduinoJson library

WiFiUDP udp; // Create a WiFiUDP object
WiFiClient client; // Create a WiFiClient object
IPAddress hostIP; // Variable to store the host IP address

void setupNetworking() {
    WiFi.mode(WIFI_STA); // Set WiFi mode to station
    WiFi.setSleep(false); // Disable WiFi sleep mode
    WiFi.begin(ssid, password); // Connect to WiFi network
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); // Wait for connection
        Serial.print(".");
    }
    Serial.println("Connected");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP()); // Print local IP address
    delay(500); // Delay for 500 milliseconds

    // Resolve the host name to an IP address
    if (!WiFi.hostByName(host, hostIP)) {
        Serial.println("Host resolution failed"); // Print error message if resolution fails
    }

#if COMM_METHOD == COMM_METHOD_UDP
    udp.begin(portNum); // Initialize UDP
#elif COMM_METHOD == COMM_METHOD_TCP
    while (!client.connect(hostIP, portNum)) {
        delay(1000); // Wait for connection
        Serial.println("Reconnecting to TCP server...");
    }
#endif
}

void sendJSON(struct MyLink *p, const char *shortAddress, const char *host, uint16_t portNum) {
    StaticJsonDocument<500> doc; // Create a JSON document
    doc["id"] = shortAddress; // Add short address to JSON document
    JsonArray links = doc.createNestedArray("links"); // Create a nested array for links
    struct MyLink *temp = p;
    while (temp->next != NULL) {
        temp = temp->next;
        JsonObject obj1 = links.createNestedObject(); // Create a nested object for each link
        obj1["a"] = temp->anchor_addr; // Add anchor address to JSON object
        char range[5];
        sprintf(range, "%.2f", temp->range[0]); // Format range as string
        obj1["r"] = range; // Add range to JSON object
    }

    char buffer[512];
    size_t len = serializeJson(doc, buffer); // Serialize JSON document to buffer
    sendData(buffer, len); // Send the JSON data
}

void sendData(const char* data, size_t len) {
#if COMM_METHOD == COMM_METHOD_SERIAL
    Serial.write(data, len); // Send data over Serial
#elif COMM_METHOD == COMM_METHOD_UDP
    udp.beginPacket(hostIP, portNum); // Start UDP packet
    udp.write(reinterpret_cast<const uint8_t*>(data), len); // Write data to UDP packet
    udp.endPacket(); // End UDP packet
#elif COMM_METHOD == COMM_METHOD_TCP
    if (client.connected()) {
        client.write(reinterpret_cast<const uint8_t*>(data), len); // Write data to TCP client
    } else {
        while (!client.connect(hostIP, portNum)) {
            delay(1000); // Wait for connection
            Serial.println("Reconnecting to TCP server...");
        }
        client.write(reinterpret_cast<const uint8_t*>(data), len); // Write data to TCP client
    }
#endif
}
