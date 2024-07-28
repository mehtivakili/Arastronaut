#ifndef UWB_H
#define UWB_H

#include <DW1000Ranging.h> // Include DW1000 Ranging library
#include "config.h"        // Include configuration file
#include "link.h"          // Include link management

void setupUWB();
void newRange();
void newDevice(DW1000Device *device);
void inactiveDevice(DW1000Device *device);
void calibrateUWBAntennaDelay();
void applyUWBAntennaDelay();

#endif // UWB_H
