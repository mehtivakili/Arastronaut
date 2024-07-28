#ifndef GLOBALS_H
#define GLOBALS_H

#include "link.h"

#if DEVICE_TYPE == IS_TAG
extern unsigned long lastUpdateTime;
extern unsigned long updateInterval; // Ensure consistent type
extern MyLink *uwb_data; // Declare uwb_data as extern
#endif

// Define USB_Packet
struct DPL {
  int8_t h1;
  int8_t h2;
  int8_t h3;
  int8_t h4;
  uint32_t TimeStamp;
  float range;
};

union {
  struct DPL Data;
  uint8_t buffer[12]; // Adjust the size as needed
} USB_Packet;

#endif // GLOBALS_H
