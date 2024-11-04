#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>
// #include <FreeRTOS.h>

// Pin definitions
const uint8_t PIN_SS = 5;   // Slave Select pin
const uint8_t PIN_RST = 27;   // Reset pin
const uint8_t PIN_IRQ = 34;   // Interrupt pin

// TDMA parameters
const uint8_t ANCHOR_ID = 1;             // Unique anchor ID (1 to NUM_ANCHORS)
const uint16_t SLOT_DURATION_MS = 100;   // Duration of each time slot in milliseconds
const uint16_t SYNC_MESSAGE_SIZE = 2;    // Size of the sync message

// Timing variables
DW1000Time syncRxTime;
DW1000Time slotOffsetTime;
DW1000Time txTime;

// Message buffers
byte rxBuffer[128];
byte txResponseMessage[] = {0xDE, 0xAD, 0xBE, 0xEF}; // Example response message

// Task handles
TaskHandle_t rxTaskHandle;
TaskHandle_t txTaskHandle;

// Semaphore for synchronization between ISR and tasks
SemaphoreHandle_t rxSemaphore;

// Function prototypes
void rxTask(void *pvParameters);
void txTask(void *pvParameters);
void handleReceived();

void setup() {
  Serial.begin(115200);

  // Initialize DW1000
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(ANCHOR_ID + 1); // Anchor's short address
  DW1000.setNetworkId(10);                // Network ID
  DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
  DW1000.commitConfiguration();

  // Set callback for reception complete
  DW1000.attachReceivedHandler(handleReceived);

  // Create a binary semaphore
  rxSemaphore = xSemaphoreCreateBinary();

  // Create tasks
  xTaskCreate(rxTask, "RxTask", 2048, NULL, 2, &rxTaskHandle);
  xTaskCreate(txTask, "TxTask", 2048, NULL, 1, &txTaskHandle);

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty loop; all action happens in tasks
}

// Task to handle reception
void rxTask(void *pvParameters) {
  for (;;) {
    // Start listening for sync messages
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.startReceive();

    // Wait for reception to complete (signaled by ISR)
    if (xSemaphoreTake(rxSemaphore, portMAX_DELAY) == pdPASS) {
      uint16_t dataLength = DW1000.getDataLength();
      DW1000.getData(rxBuffer, dataLength);

      // Check if it's a sync message
      if (dataLength == SYNC_MESSAGE_SIZE && rxBuffer[0] == 0xCA && rxBuffer[1] == 0xFE) {
        Serial.println("Sync message received.");
        // Get the receive timestamp
        DW1000.getReceiveTimestamp(syncRxTime);

        // Calculate the transmission time for this anchor's slot
        // slotOffsetTime = DW1000Time((uint64_t)(SLOT_DURATION_MS * (ANCHOR_ID - 1)) * 1e3); // Convert ms to DW1000 time units
        slotOffsetTime = DW1000Time((int64_t)(SLOT_DURATION_MS * (ANCHOR_ID - 1) * 1000));
        txTime = syncRxTime + slotOffsetTime;
        // txTime = syncRxTime + slotOffsetTime;

        // Notify txTask to send the response
        xTaskNotifyGive(txTaskHandle);
      }
    }
  }
}

// Task to handle transmission
void txTask(void *pvParameters) {
  for (;;) {
    // Wait for notification from rxTask
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Prepare delayed transmission
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setDelay(txTime - DW1000Time()); // Schedule transmission at txTime
    DW1000.setData(txResponseMessage, sizeof(txResponseMessage));
    DW1000.startTransmit();
    Serial.print("Scheduled response at time: ");
    Serial.println(txTime.getAsMicroSeconds());
  }
}

void handleReceived() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  // Give semaphore to unblock rxTask
  xSemaphoreGiveFromISR(rxSemaphore, &xHigherPriorityTaskWoken);
  // Context switch if necessary
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
