// main_anchor.cpp
#include <Arduino.h>
#include "DW1000Ranging.h"
#include "DW1000Device.h"

// Pin Definitions
const uint8_t RST_PIN = 27;    // Reset Pin (ensure different from Tag)
const uint8_t SS_PIN = 5;      // Slave Select Pin
const uint8_t IRQ_PIN = 34;    // Interrupt Pin
volatile bool interruptFlag = false;
byte mode[] = { 0x88, 0xCC, 0x01, 0x00 }; // Example Mode Configuration

// Instantiate the DW1000RangingClass object
// DW1000RangingClass DW1000Ranging;

// ISR to set the interrupt flag
void IRAM_ATTR dw1000_isr() {
    interruptFlag = true;
}

// Callback function when a new range is received
void handleNewRange() {
    Serial.println("Anchor: New range data received.");
    // Implement additional processing as needed
}

// Callback function when a new device is detected
void handleNewDevice(DW1000Device* device) {
    Serial.print("Anchor: New device connected - ");
    char addressStr[20];
    device->getPrintableShortAddress(addressStr); // Ensure this method is implemented
    Serial.println(addressStr);
}

void setup() {
    // Initialize Serial communication for debugging
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for Serial to be ready
    }
    Serial.println("Initializing DW1000 Anchor (Slave)...");

    // Initialize DW1000 communication
    DW1000Ranging.initCommunication(RST_PIN, SS_PIN, IRQ_PIN);

    // Configure the network
    uint16_t deviceAddress = 0x5678; // Unique Anchor Address
    uint16_t networkId = 0xDECA;     // Common Network ID

    // Attach callback handlers
    DW1000Ranging.setHandleNewRange(handleNewRange);
    DW1000Ranging.setHandleNewDevice(handleNewDevice);

    // Initialize TDMA
    DW1000Ranging.initTDMA();

    // Start as Anchor (Slave)
    char anchorAddress[] = "ANCH001"; // Replace with actual Anchor Address
    bool randomShortAddress = false;   // Typically, Anchors have fixed addresses
    DW1000Ranging.startAsAnchor(anchorAddress, mode, randomShortAddress);

    // Attach Interrupt Service Routine (ISR) for DW1000
    attachInterrupt(digitalPinToInterrupt(IRQ_PIN), dw1000_isr, RISING); // Trigger on rising edge

    Serial.println("DW1000 Anchor (Slave) initialized successfully.");
}

void loop() {
    if (interruptFlag) {
        interruptFlag = false;
        DW1000Ranging.handleInterrupt();
    }

    DW1000Ranging.loop();

    // Optional: Add a small delay to prevent watchdog from triggering
    delay(1);
}
