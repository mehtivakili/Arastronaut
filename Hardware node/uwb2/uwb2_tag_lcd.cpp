#include <SPI.h>
#include "DW1000Ranging.h"
#include <lcdgfx.h>
#include <map>

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 5;   // spi select pin

// LCD settings
DisplaySSD1306_128x64_I2C lcd(-1); // Use -1 for I2C interface

bool updateLCD = false;
std::map<String, std::pair<unsigned long, float>> activeDevices; // Store timestamp and range

void inactiveDevice(DW1000Device *device);
void newDevice(DW1000Device *device);
void newRange();
void updateDisplay();
void updateRangeOnly(int row, float range);

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Initialize LCD
    lcd.begin();
    lcd.setFixedFont(ssd1306xled_font6x8);
    lcd.clear();
    lcd.printFixed(0, 0, "Initializing...", STYLE_NORMAL);

    // Init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pin

    // Define the sketch as anchor
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    // Start the module as a tag
    DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_LOWPOWER);

    // Display ready message on LCD
    lcd.clear();
}

void loop()
{
    DW1000Ranging.loop();

    unsigned long currentMillis = millis();

    // Remove inactive devices after 5 seconds
    for (auto it = activeDevices.begin(); it != activeDevices.end(); ) {
        if (currentMillis - it->second.first > 5000) {
            Serial.print("Removing inactive device: ");
            Serial.println(it->first);
            it = activeDevices.erase(it);
            updateLCD = true;
        } else {
            ++it;
        }
    }

    if (updateLCD) {
        updateDisplay();
        updateLCD = false;
    }

    yield(); // Reset watchdog timer
}

void newRange()
{
    DW1000Device *device = DW1000Ranging.getDistantDevice();
    if (device) {
        String shortAddress = String(device->getShortAddress(), HEX);
        float range = device->getRange();
        if (activeDevices.find(shortAddress) != activeDevices.end()) {
            activeDevices[shortAddress] = { millis(), range };
            int row = std::distance(activeDevices.begin(), activeDevices.find(shortAddress));
            updateRangeOnly(row, range);
        } else {
            activeDevices[shortAddress] = { millis(), range };
            updateLCD = true;
        }
        Serial.print("Updated range for device: ");
        Serial.print(shortAddress);
        Serial.print(" - Range: ");
        Serial.print(range);
        Serial.println(" m");
    }
}

void newDevice(DW1000Device *device)
{
    String shortAddress = String(device->getShortAddress(), HEX);
    activeDevices[shortAddress] = { millis(), 0.0f }; // Initial range is 0.0
    Serial.print("New device added: ");
    Serial.println(shortAddress);
    updateLCD = true;
}

void inactiveDevice(DW1000Device *device)
{
    String shortAddress = String(device->getShortAddress(), HEX);
    Serial.print("Device inactive: ");
    Serial.println(shortAddress);
    activeDevices.erase(shortAddress);
    updateLCD = true;
}

void updateDisplay()
{
    lcd.clear();

    int row = 0;
    for (const auto& pair : activeDevices) {
        char buffer[32];

        // Display short address
        snprintf(buffer, sizeof(buffer), "Short: %s", pair.first.c_str());
        lcd.printFixed(0, row * 16, buffer, STYLE_NORMAL);

        // Display range
        snprintf(buffer, sizeof(buffer), "Range: ");
        lcd.printFixed(0, row * 16 + 8, buffer, STYLE_NORMAL);

        // Only print the numeric range value next to "Range:" without clearing "m"
        updateRangeOnly(row, pair.second.second);

        row++;
    }
}

void updateRangeOnly(int row, float range)
{
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.2f", range);
    lcd.printFixed(48, row * 16 + 8, buffer, STYLE_NORMAL);
}
