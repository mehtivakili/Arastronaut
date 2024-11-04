#include <SPI.h>
#include "DW1000.h"

const uint8_t PIN_RST = 27; // Reset pin
const uint8_t PIN_IRQ = 34; // IRQ pin
const uint8_t PIN_SS = 5;   // SS pin

volatile bool received = false;
void handleReceived();

void setup() {
  Serial.begin(115200);
  delay(1000);

  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);

  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.commitConfiguration();

  DW1000.attachReceivedHandler(handleReceived);

  DW1000.newReceive();
  DW1000.setDefaults();
  DW1000.receivePermanently(true);
  DW1000.startReceive();

  Serial.println("Slave: Ready to receive");
}

void loop() {
  if (received) {
    received = false;
    char message[128] = {0};
    DW1000.getData((byte*)message, 128);
    Serial.print("Slave: Message received - ");
    Serial.println(message);

    // Restart receive mode
    DW1000.newReceive();
    DW1000.setDefaults();
    DW1000.receivePermanently(true);
    DW1000.startReceive();
  }
  // delay(20); // Ensure the WDT is fed
}

void IRAM_ATTR handleReceived() {
  received = true;
  // Do not call Serial.print or other non-ISR safe functions here
}
