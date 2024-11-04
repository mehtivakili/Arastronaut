#include <SPI.h>
#include <DW1000.h>

// Pin definitions
const uint8_t PIN_SS = 10;   // Slave Select pin
const uint8_t PIN_RST = 9;   // Reset pin
const uint8_t PIN_IRQ = 2;   // Interrupt pin

// TDMA parameters
const uint8_t NUM_ANCHORS = 4;         // Number of anchors (adjust as needed)
const uint16_t SLOT_DURATION_MS = 100; // Duration of each time slot in milliseconds
const uint16_t FRAME_PERIOD_MS = NUM_ANCHORS * SLOT_DURATION_MS + 50; // Total frame duration

// Message buffers
byte txSyncMessage[] = {0xCA, 0xFE}; // Example sync message
byte rxBuffer[128];

void setup() {
  Serial.begin(115200);
  // Initialize DW1000
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(1); // Tag's short address
  DW1000.setNetworkId(10);    // Network ID
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();

  // Set callback for transmission complete
  DW1000.attachSentHandler(handleSent);

  // Begin periodic synchronization
  startSyncTimer();
}

void loop() {
  // Main loop does nothing; synchronization handled by timer and interrupt
}

void startSyncTimer() {
  // Send synchronization message periodically
  sendSyncMessage();
  // Schedule next synchronization
  scheduleNextSync();
}

void scheduleNextSync() {
  // Use a timer or delay to schedule the next sync
  // For simplicity, using delay here (not precise for real applications)
  delay(FRAME_PERIOD_MS);
  startSyncTimer();
}

void sendSyncMessage() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  DW1000.setData(txSyncMessage, sizeof(txSyncMessage));
  DW1000.startTransmit();
}

void handleSent() {
  Serial.println("Sync message sent.");
  // Optionally, start listening for anchor responses here
}
