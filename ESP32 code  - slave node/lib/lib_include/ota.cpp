#include <ArduinoOTA.h> // Include ArduinoOTA library
#include "ota.h" // Include the OTA header file

void setupOTA() {
  ArduinoOTA.setHostname("esp32-ota"); // Set the hostname for OTA updates

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch"; // Updating sketch
    } else { // U_SPIFFS
      type = "filesystem"; // Updating filesystem
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type); // Print the update type
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd"); // Print end message
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100))); // Print update progress
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error); // Print error code
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed"); // Print authentication error
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed"); // Print begin error
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed"); // Print connection error
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed"); // Print receive error
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed"); // Print end error
    }
  });

  ArduinoOTA.begin(); // Start OTA service
}

void handleOTA() {
  ArduinoOTA.handle(); // Handle OTA updates
}
