#include <SPI.h>
#include "DW1000Ranging.h"

// SPI pins
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// Connection pins
const uint8_t PIN_RST = 27; // Reset pin
const uint8_t PIN_IRQ = 34; // IRQ pin (must be interrupt-capable)
const uint8_t PIN_SS = 5;   // SPI select pin

// Device address (unique 16-bit address)
const uint16_t DEVICE_ADDRESS = 0x1234;

// Network ID (must be the same for all devices)
const uint16_t NETWORK_ID = 0xDECA;

// Operating mode (pointer to the mode array)
const byte* MODE = DW1000.MODE_LONGDATA_RANGE_LOWPOWER;

// EUI-64 address (unique for each device)
// Ensure this is a mutable array, not a const array
char TAG_EUI64[] = "7D:00:22:EA:82:60:3B:9C";

// Callback function declarations
void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting Tag (Master)");

  // Initialize SPI
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Initialize DW1000 communication
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pins

  // Configure network with device address, network ID, and mode
  DW1000Ranging.configureNetwork(DEVICE_ADDRESS, NETWORK_ID, MODE);

  // Attach callback functions
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // Start as Tag (Master)
  // Pass all three required arguments: address, mode, and randomShortAddress (optional)
  DW1000Ranging.startAsTag(TAG_EUI64, MODE, true);

  Serial.println("Tag started");
}

void loop() {
  // Main loop for the Tag (Master)
  DW1000Ranging.loopMaster();
}

// Callback function implementations
void newRange() {
  DW1000Device *device = DW1000Ranging.getDistantDevice();
  Serial.print("From: ");
  Serial.print(device->getShortAddress(), HEX);
  Serial.print("\tRange: ");
  Serial.print(device->getRange(), 4);
  Serial.print(" m\tRX Power: ");
  Serial.print(device->getRXPower(), 2);
  Serial.println(" dBm");
}

void newDevice(DW1000Device *device) {
  Serial.print("New device discovered: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("Device inactive: ");
  Serial.println(device->getShortAddress(), HEX);
}
