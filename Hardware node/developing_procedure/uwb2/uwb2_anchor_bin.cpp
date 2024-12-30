#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// leftmost two bytes below will become the "short address"
char anchor_addr[] = "84:00:5B:D5:A9:9A:E2:9C"; //#4

// calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16527; 

// calibration distance
float dist_m = 7.19; // meters

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 5;   // spi select pin

// Track the start time
unsigned long startTime;

void inactiveDevice(DW1000Device *device);
void newDevice(DW1000Device *device);
void newRange();

void setup() {
  Serial.begin(115200);
  delay(1000); // wait for serial monitor to connect
  Serial.println("Anchor config and start");
  Serial.print("Antenna delay ");
  Serial.println(Adelay);
  Serial.print("Calibration distance ");
  Serial.println(dist_m);

  // Init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pin

  // Set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // Start the module as an anchor
  DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);

  // Record the start time
  startTime = millis();
}

void loop() {
  DW1000Ranging.loop();
}

void newRange() {
  unsigned long currentTime = millis();  // Get current time
  float elapsedTime = (currentTime - startTime) / 1000.0; // Calculate elapsed time in seconds

  // Convert the time to binary
  byte timeBytes[sizeof(float)];
  memcpy(timeBytes, &elapsedTime, sizeof(float));

  // Get the range distance
  float dist = DW1000Ranging.getDistantDevice()->getRange();

  // Convert the distance to binary
  byte distBytes[sizeof(float)];
  memcpy(distBytes, &dist, sizeof(float));

  // Define the separator \abc in binary
  const char separator[] = "\abc";

  // Send the time and distance as binary over serial
  Serial.write(timeBytes, sizeof(timeBytes));  // Send the time stamp
  Serial.write(distBytes, sizeof(distBytes));  // Send the distance

  // Send the separator in binary
  Serial.write(separator, strlen(separator));  // Send \abc as binary
}

void newDevice(DW1000Device *device) {
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
