// MasterNode.ino

#include <SPI.h>
#include <DW1000.h>

// Connection pins
const uint8_t PIN_RST = 27; // Reset pin
const uint8_t PIN_IRQ = 34; // IRQ pin
const uint8_t PIN_SS = 5;   // SPI select pin

// Define Data Rates and Pulse Frequencies (Adjust based on your library)
#define TRX_RATE_6800KBPS 2      // Example value; verify with your library's documentation
#define TX_PULSE_FREQ_64MHZ 1    // Example value; verify with your library's documentation

// TDMA Parameters
#define NUM_SLOTS         1          // Number of Slave Nodes (Set to 1 for simplicity)
#define SLOT_DURATION_US  50000      // Slot duration in microseconds (50 ms) -> 20 Hz
#define SYNC_INTERVAL_US  (SLOT_DURATION_US * (NUM_SLOTS + 1)) // Total cycle duration (100 ms)

// Slot Identifiers
enum SlotID {
  SYNC_SLOT = 0,
  SLAVE_1_SLOT
};

// Timing Variables
unsigned long lastSyncTime = 0;

// Function Prototypes
void sendDataToSlave(uint8_t slaveID, const char* message);
void sendSyncFrame();

void setup() {
  Serial.begin(115200);
  
  // Initialize DW1000
  DW1000.begin(PIN_IRQ, PIN_RST); // Corrected: Only IRQ and RST
  DW1000.select(PIN_SS);
  DW1000.setDefaults();
  
  // Set Data Rate and PRF
  DW1000.setDataRate(TRX_RATE_6800KBPS); // 6.8 Mbps
  DW1000.setPulseFrequency(TX_PULSE_FREQ_64MHZ); // 64 MHz PRF
  DW1000.commitConfiguration();
  
  // Start Receiving
  DW1000.newReceive();
  DW1000.startReceive();
  
  // Initialize Timing
  lastSyncTime = micros();
  
  Serial.println("Master: Initialization Complete.");
}

void loop() {
  unsigned long currentTime = micros();
  
  // Check if it's time to send a Sync Frame
  if (currentTime - lastSyncTime >= SYNC_INTERVAL_US) {
    sendSyncFrame();
    lastSyncTime = currentTime;
  }
  
  // Calculate Current Slot
  unsigned long elapsedTime = currentTime - lastSyncTime;
  unsigned int currentSlot = (elapsedTime / SLOT_DURATION_US) % (NUM_SLOTS + 1);
  
  // Handle Slot Actions
  switch(currentSlot) {
    case SYNC_SLOT:
      // Already handled by sendSyncFrame
      break;
    case SLAVE_1_SLOT:
      sendDataToSlave(1, "Hello Slave 1");
      break;
    default:
      // Idle or Handle Unexpected Slots
      break;
  }
  
  // Polling for Transmission Completion
  if (DW1000.isTransmitDone()) {
    Serial.println("Master: Transmission Complete.");
    // Optional: Add any post-transmission logic here
  }
  
  // Polling for Received Data
  if (DW1000.isReceiveDone()) {
    byte receiveBuffer[128];
    DW1000.getData(receiveBuffer, DW1000.getDataLength());
    
    Serial.print("Master: Data Received: ");
    Serial.println((char*)receiveBuffer);
    
    // Restart Receiving
    DW1000.newReceive();
    DW1000.startReceive();
  }
}

// Function to send a synchronization frame
void sendSyncFrame() {
  byte syncData[] = "SYNC";
  DW1000.newTransmit();
  DW1000.setData(syncData, sizeof(syncData));
  DW1000.startTransmit();
  Serial.println("Master: Sent Synchronization Frame.");
}

// Function to send data to a specific Slave
void sendDataToSlave(uint8_t slaveID, const char* message) {
  DW1000.newTransmit();
  DW1000.setData((byte*)message, strlen(message) + 1); // +1 for null terminator
  DW1000.startTransmit();
  Serial.print("Master: Sent Data to Slave ");
  Serial.println(slaveID);
}
