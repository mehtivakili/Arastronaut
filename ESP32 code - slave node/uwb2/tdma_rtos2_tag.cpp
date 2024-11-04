#include <Arduino.h>
#include <SPI.h>
#include <DW1000.h>

// Pin definitions
const uint8_t PIN_SS = 5;    // Slave Select pin
const uint8_t PIN_RST = 27;  // Reset pin
const uint8_t PIN_IRQ = 34;  // Interrupt pin

// Message buffers
byte txPollMessage[] = {0x81, 0x00};   // Poll message identifier
byte rxResponseMessage[128];           // Buffer for received response
DW1000Time pollTxTime;
DW1000Time responseRxTime;
DW1000Time timeOfFlight;
float distance;

// Constants
const float SPEED_OF_LIGHT = 299792458.0;                  // Speed of light in meters per second
const double DW1000_TIME_UNITS_TO_SECONDS = 1.0 / 499200000.0; // Seconds per UWB time unit
const double DW1000_TIME_UNITS_PER_MICROSECOND = 499.2;       // UWB time units per microsecond
const uint16_t REPLY_DELAY_US = 5000;                       // Must match the anchor's reply delay

// Task handle
TaskHandle_t rangingTaskHandle;

// Semaphores
SemaphoreHandle_t txCompleteSemaphore;
SemaphoreHandle_t rxCompleteSemaphore;

// Function prototypes
void rangingTask(void *pvParameters);
void handleSent();
void handleReceived();

void setup() {
  Serial.begin(115200);

  // Initialize DW1000
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);
  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.setDeviceAddress(1); // Tag's short address
  DW1000.setNetworkId(10);    // Network ID
  DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_LOWPOWER);
  DW1000.commitConfiguration();

  // Set callbacks
  DW1000.attachSentHandler(handleSent);
  DW1000.attachReceivedHandler(handleReceived);

  // Create semaphores
  txCompleteSemaphore = xSemaphoreCreateBinary();
  rxCompleteSemaphore = xSemaphoreCreateBinary();

  // Create task
  xTaskCreate(rangingTask, "RangingTask", 4096, NULL, 2, &rangingTaskHandle);

  Serial.println("Setup complete");
}

void loop() {
  // Empty loop, task handles everything
}

void rangingTask(void *pvParameters) {
  const uint16_t POLL_INTERVAL_MS = 1000;         // Interval between polls
  const uint16_t RESPONSE_TIMEOUT_MS = 100;       // Timeout for response in milliseconds

  for (;;) {
    Serial.println("Tag: Sending poll message");
    DW1000.newTransmit();
    DW1000.setDefaults();
    DW1000.setData(txPollMessage, sizeof(txPollMessage));
    DW1000.startTransmit();

    // Wait for transmission to complete
    if (xSemaphoreTake(txCompleteSemaphore, portMAX_DELAY) == pdPASS) {
      Serial.println("Tag: Poll message sent");

      // Get transmit timestamp
      DW1000.getTransmitTimestamp(pollTxTime);

      // Start listening for response
      DW1000.newReceive();
      DW1000.setDefaults();
      DW1000.startReceive();

      // Wait for reception to complete with timeout
      if (xSemaphoreTake(rxCompleteSemaphore, pdMS_TO_TICKS(RESPONSE_TIMEOUT_MS)) == pdPASS) {
        // Get receive timestamp
        DW1000.getReceiveTimestamp(responseRxTime);

        // Get received data
        uint16_t dataLength = DW1000.getDataLength();
        DW1000.getData(rxResponseMessage, dataLength);

        // Check if it's a response message
        if (dataLength >= 2 && rxResponseMessage[0] == 0x82) {
          Serial.println("Tag: Received response from anchor");

          // Calculate round-trip time
          uint64_t roundTripTime = responseRxTime.getTimestamp() - pollTxTime.getTimestamp();

          // Calculate reply delay in UWB time units
          uint64_t replyDelayTimeUnits = (uint64_t)(REPLY_DELAY_US * DW1000_TIME_UNITS_PER_MICROSECOND); // 5000 * 499.2 = 2,496,000 UWB time units

          // Calculate time of flight
          uint64_t timeOfFlightUnits = (roundTripTime - replyDelayTimeUnits) / 2;

          // Calculate distance
          float distance = timeOfFlightUnits * DW1000_TIME_UNITS_TO_SECONDS * SPEED_OF_LIGHT;

          Serial.print("Tag: Distance to anchor: ");
          Serial.print(distance, 4); // 4 decimal places
          Serial.println(" meters");
        } else {
          Serial.println("Tag: Received unknown message");
        }
      } else {
        Serial.println("Tag: No response received");
      }
    } else {
      Serial.println("Tag: Transmission failed");
    }

    // Wait before next poll
    vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
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
