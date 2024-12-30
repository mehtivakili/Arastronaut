#include <SPI.h>
#include "DW1000.h"

const uint8_t PIN_RST = 27; // Reset pin
const uint8_t PIN_IRQ = 34; // IRQ pin (unused here)
const uint8_t PIN_SS = 5;   // SS pin

void setup() {
  Serial.begin(115200);
  delay(1000);

  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.select(PIN_SS);

  DW1000.newConfiguration();
  DW1000.setDefaults();
  DW1000.commitConfiguration();

  Serial.println("Master: Ready to send");
}

void loop() {
  DW1000.newTransmit();
  DW1000.setDefaults();
  const char message[] = "Hello from Master";
  DW1000.setData((byte*)message, sizeof(message));
  DW1000.startTransmit();
  Serial.println("Master: Message sent");
  delay(20);
}
