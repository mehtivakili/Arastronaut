#ifndef DISPLAY_H
#define DISPLAY_H

#include "lcdgfx.h"

void initDisplay(DisplaySSD1306_128x64_I2C &display);
void updateDisplay(DisplaySSD1306_128x64_I2C &display, const char *shortAddress, const char *deviceAddress);

#endif // DISPLAY_H
