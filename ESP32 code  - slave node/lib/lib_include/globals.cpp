// globals.cpp
#include "globals.h"

// Define the global variables
#if DEVICE_TYPE == IS_TAG
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 200; // Ensure consistent type
MyLink *uwb_data = nullptr;
#endif
