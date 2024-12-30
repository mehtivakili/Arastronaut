// SlaveNode.ino

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
#define SLOT_DURATION_US  50000      // Slot duration in microseconds (50 ms) -> 20 Hz

// Assign a unique Slave ID (1 to NUM_SLOTS)
#define SLAVE_ID          1          // Change to 2, 3, ... for other Slaves if needed

// Timing Variables
unsigned long lastSyncTime = 0;
bool isSynchronized = false;

// Function Prototypes
void sendDataToMaster(const char* message);

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
  
  Serial.println("Slave: Initialization Complete.");
}

void loop() {
  unsigned long currentTime = micros();
  
  if (isSynchronized) {
    // Calculate Current Slot
    unsigned long elapsedTime = currentTime - lastSyncTime;
    unsigned int currentSlot = (elapsedTime / SLOT_DURATION_US) % (1 + 1); // NUM_SLOTS =1
    
    // Determine if it's this Slave's Slot
    if (currentSlot == SLAVE_ID) {
      // Transmit Data during this slot
      sendDataToMaster("Hello Master from Slave");
      // Update lastSyncTime to prevent immediate retransmission
      lastSyncTime = currentTime;
    }
  }
  
  // Polling for Received Data
  if (DW1000.isReceiveDone()) {
    byte receiveBuffer[128];
    DW1000.getData(receiveBuffer, DW1000.getDataLength());
    
    // Check if it's a synchronization frame
    if (strcmp((char*)receiveBuffer, "SYNC") == 0) {
      Serial.println("Slave: Synchronization Frame Received.");
      isSynchronized = true;
      lastSyncTime = micros(); // Reset timing
    } else {
      Serial.print("Slave: Data Received: ");
      Serial.println((char*)receiveBuffer);
      
      // Optionally respond or process the data
    }
    
    // Restart Receiving
    DW1000.newReceive();
    DW1000.startReceive();
  }
  
  // Polling for Transmission Completion
  if (DW1000.isTransmitDone()) {
    Serial.println("Slave: Transmission Complete.");
    // Optional: Add any post-transmission logic here
  }
}

// Function to send data to Master
void sendDataToMaster(const char* message) {
  DW1000.newTransmit();
  DW1000.setData((byte*)message, strlen(message) + 1); // +1 for null terminator
  DW1000.startTransmit();
  Serial.println("Slave: Sent Data to Master.");
}
