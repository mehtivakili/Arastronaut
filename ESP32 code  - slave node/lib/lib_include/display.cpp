#include "display.h" // Include the display header file

void initDisplay(DisplaySSD1306_128x64_I2C &display) {
  display.setFixedFont(ssd1306xled_font6x8); // Set the display font
  display.begin(); // Initialize the display
  display.clear(); // Clear the display
}

void updateDisplay(DisplaySSD1306_128x64_I2C &display, const char *shortAddress, const char *deviceAddress) {
  display.printFixed(0, 8, deviceAddress, STYLE_NORMAL); // Print the device address
  display.printFixed(0, 16, shortAddress, STYLE_NORMAL); // Print the short address
}
