#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>
#include <FreeRTOS.h>

// Pin definitions
const uint8_t PIN_SS = 10;   // Slave Select pin
const uint8_t PIN_RST = 9;   // Reset pin
const uint8_t PIN_IRQ = 2;   // Interrupt pin

// TDMA parameters
const uint8_t NUM_ANCHORS = 4;         // Number of anchors
const uint16_t SLOT_DURATION_MS = 100; // Duration of each time slot in milliseconds
const uint16_t FRAME_PERIOD_MS = NUM_ANCHORS * SLOT_DURATION_MS + 50; // Total frame duration

// Message buffers
byte txSyncMessage[] = {0xCA, 0xFE}; // Example sync message

// Task handles
TaskHandle_t syncTaskHandle;
TaskHandle_t txTaskHandle;

// Function prototypes
void syncTask(void *pvParameters);
void txTask(void *pvParameters);
void handleSent();

// Queue for synchronization between tasks
QueueHandle_t txQueue;

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

  // Create a queue for communication between tasks
  txQueue = xQueueCreate(5, sizeof(byte *));

  // Create tasks
  xTaskCreate(syncTask, "SyncTask", 2048, NULL, 2, &syncTaskHandle);
  xTaskCreate(txTask, "TxTask", 2048, NULL, 1, &txTaskHandle);

  // Start the scheduler
  vTaskStartScheduler();
}

void loop() {
  // Empty loop; all action happens in tasks
}

// Task to send synchronization messages periodically
void syncTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(FRAME_PERIOD_MS);

  for (;;) {
    // Send sync message
    byte *message = txSyncMessage;
    xQueueSend(txQueue, &message, portMAX_DELAY);

    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task to handle transmissions
void txTask(void *pvParameters) {
  byte *message;

  for (;;) {
    // Wait for a message to transmit
    if (xQueueReceive(txQueue, &message, portMAX_DELAY) == pdPASS) {
      // Transmit the message
      DW1000.newTransmit();
      DW1000.setDefaults();
      DW1000.setData(message, sizeof(txSyncMessage));
      DW1000.startTransmit();

      // Wait for transmission to complete (handled in interrupt)
    }
  }
}

void handleSent() {
  Serial.println("Sync message sent.");
  // Optionally notify tasks or handle post-transmission actions
}
