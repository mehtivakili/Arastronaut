#include <Wire.h>
#include <Arduino.h>

void setup() {
  Serial.begin(115200);  // Initialize serial communication for debugging
  Wire.begin();          // Initialize I2C (SDA = GPIO 21, SCL = GPIO 22 on ESP32)
  
  Serial.println("I2C Scanner");
}

void loop() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning...");

  for(address = 1; address < 127; address++) {
    // The I2C scanner uses the return value of
    // Wire.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("Done.\n");

  delay(5000); // Wait 5 seconds before next scan
}
