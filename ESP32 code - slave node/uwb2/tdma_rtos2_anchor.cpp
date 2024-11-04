#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>

// Pin definitions
const uint8_t PIN_SS = 5;    // Slave Select pin
const uint8_t PIN_RST = 27;  // Reset pin
const uint8_t PIN_IRQ = 34;  // Interrupt pin

// Message buffers
byte rxPollMessage[128];                 // Buffer to receive the poll message
byte txResponseMessage[] = {0x82, 0x00}; // Response message identifier
DW1000Time pollRxTime;
DW1000Time responseTxTime;

// Constants
const float SPEED_OF_LIGHT = 299792458.0; // Speed of light in meters per second
const uint16_t REPLY_DELAY_US = 5000;     // Reply delay in microseconds

// Task handles
TaskHandle_t rxTaskHandle;
TaskHandle_t txTaskHandle;

// Semaphores
SemaphoreHandle_t rxCompleteSemaphore;
SemaphoreHandle_t txCompleteSemaphore;

// Function prototypes
void rxTask(void *pvParameters);
void txTask(void *pvParameters);
void handleSent();
void handleReceived();

void setup() {
  Serial.begin(115200);

  // Initialize DW1000
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(2);  // Anchor's short address
  DW1000.setNetworkId(10);     // Network ID
  DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
  DW1000.commitConfiguration();

  // Set callbacks
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);

  // Create semaphores
  rxCompleteSemaphore = xSemaphoreCreateBinary();
  txCompleteSemaphore = xSemaphoreCreateBinary();

  // Create tasks
  xTaskCreate(rxTask, "RxTask", 4096, NULL, 2, &rxTaskHandle);
  xTaskCreate(txTask, "TxTask", 4096, NULL, 1, &txTaskHandle);

  Serial.println("Anchor setup complete");
}

void loop() {
  // Empty loop, tasks handle everything
}

void rxTask(void *pvParameters) {
  for (;;) {
    Serial.println("Anchor: Listening for poll message");

    // Start listening for poll messages
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.startReceive();

    // Wait for reception to complete
    if (xSemaphoreTake(rxCompleteSemaphore, portMAX_DELAY) == pdPASS) {
      uint16_t dataLength = DW1000.getDataLength();
      DW1000.getData(rxPollMessage, dataLength);

      // Check if it's a poll message
      if (dataLength >= 2 && rxPollMessage[0] == 0x81) {
        Serial.println("Anchor: Poll message received");

        // Get receive timestamp
        DW1000.getReceiveTimestamp(pollRxTime);

        // Notify txTask to send response
        xTaskNotifyGive(txTaskHandle);
      } else {
        Serial.println("Anchor: Unknown message received");
      }
    } else {
      Serial.println("Anchor: Reception failed");
    }
  }
}

void txTask(void *pvParameters) {
  for (;;) {
    // Wait for notification from rxTask
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    Serial.println("Anchor: Sending response message");

    // Prepare the response message
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData(txResponseMessage, sizeof(txResponseMessage));

    // Calculate delayed transmit time (absolute time)
    uint64_t pollRxTimeUs = pollRxTime.getAsMicroSeconds();
    uint64_t responseTxTimeUs = pollRxTimeUs + REPLY_DELAY_US;

    DW1000Time txDelayTime((int64_t)responseTxTimeUs);

    // Set the delayed transmit time
    DW1000.setDelay(txDelayTime);

    // Transmit the response
    DW1000.startTransmit(); // No arguments

    // Wait for transmission to complete
    if (xSemaphoreTake(txCompleteSemaphore, portMAX_DELAY) == pdPASS) {
      Serial.println("Anchor: Response message sent");

      // Get transmission timestamp
      DW1000.getTransmitTimestamp(responseTxTime);

      // Print timestamps for debugging
      Serial.print("Anchor: Poll RX time (microseconds): ");
      Serial.println(pollRxTime.getAsMicroSeconds());
      Serial.print("Anchor: Response TX time (microseconds): ");
      Serial.println(responseTxTime.getAsMicroSeconds());

      // Calculate time of flight (round-trip time minus reply delay)
      uint64_t roundTripTimeUs = responseTxTime.getAsMicroSeconds() - pollRxTime.getAsMicroSeconds();
      uint64_t tofUs = (roundTripTimeUs - REPLY_DELAY_US) / 2;

      // Calculate distance
      float distance = (tofUs * 1e-6 * SPEED_OF_LIGHT);

      // Print the calculated distance
      Serial.print("Anchor: Calculated Distance: ");
      Serial.print(distance, 4); // 4 decimal places
      Serial.println(" meters");

      // Debug information for tuning
      Serial.print("Anchor: Time of Flight (microseconds): ");
      Serial.println(tofUs);
    } else {
      Serial.println("Anchor: Transmission failed");
    }
  }
}

void handleSent() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(txCompleteSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void handleReceived() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(rxCompleteSemaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
