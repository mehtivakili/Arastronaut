#include <SPI.h>
#include "DW1000Ranging.h"
#include <lcdgfx.h>
#include <Wire.h>
#include "MPU9250.h"
#include "MadgwickAHRS.h"
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

MPU9250 IMU(Wire, 0x68); // IMU instance
Madgwick filter;         // Madgwick filter instance

float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; // Orientation variables

TaskHandle_t uwbTaskHandle = NULL;
TaskHandle_t orientationTaskHandle = NULL;

void inactiveDevice(DW1000Device *device);
void newDevice(DW1000Device *device);
void newRange();
void updateDisplay();
void updateRangeOnly(int row, float range);
void orientationTask(void *parameter);
void uwbTask(void *parameter);

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

    // Initialize MPU9250 IMU
    if (IMU.begin() < 0) {
        Serial.println("IMU initialization failed!");
        while (1)
            ;
    }
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);

    // Initialize Madgwick filter
    filter.begin(100); // 100 Hz sampling rate

    // Create UWB task on core 1
    xTaskCreatePinnedToCore(
        uwbTask,            // Task function
        "UWB Task",        // Task name
        4096,              // Stack size (bytes)
        NULL,              // Parameter
        1,                 // Priority
        &uwbTaskHandle,    // Task handle
        1);                // Core 1

    // Create Orientation task on core 1
    xTaskCreatePinnedToCore(
        orientationTask,       // Task function
        "Orientation Task",   // Task name
        4096,                 // Stack size (bytes)
        NULL,                 // Parameter
        1,                    // Priority
        &orientationTaskHandle,// Task handle
        1);                   // Core 1

    if (uwbTaskHandle == NULL || orientationTaskHandle == NULL)
    {
        Serial.println("Failed to create tasks!");
        while (1)
            ;
    }

    // Display ready message on LCD
    lcd.clear();
}

void loop()
{
    // Let tasks handle everything
    delay(1000); // Allow tasks to operate
}

void uwbTask(void *parameter)
{
    while (1)
    {
        DW1000Ranging.loop();

        unsigned long currentMillis = millis();

        // Remove inactive devices after 5 seconds
        for (auto it = activeDevices.begin(); it != activeDevices.end();)
        {
            if (currentMillis - it->second.first > 5000)
            {
                Serial.print("Removing inactive device: ");
                Serial.println(it->first);
                it = activeDevices.erase(it);
                updateLCD = true;
            }
            else
            {
                ++it;
            }
        }

        vTaskDelay(10 / portTICK_PERIOD_MS); // Prevent task starvation
    }
}

void orientationTask(void *parameter)
{
    while (1)
    {
        IMU.readSensor();

        // Retrieve sensor data
        float ax = IMU.getAccelX_mss();
        float ay = IMU.getAccelY_mss();
        float az = IMU.getAccelZ_mss();
        float gx = IMU.getGyroX_rads() * 180.0f / PI; // Convert to degrees/sec
        float gy = IMU.getGyroY_rads() * 180.0f / PI; // Convert to degrees/sec
        float gz = IMU.getGyroZ_rads() * 180.0f / PI; // Convert to degrees/sec
        float mx = IMU.getMagX_uT();
        float my = IMU.getMagY_uT();
        float mz = IMU.getMagZ_uT();

        // Update Madgwick filter
        filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

        // Compute roll, pitch, and yaw
        roll = atan2f(2.0f * (filter.q0 * filter.q1 + filter.q2 * filter.q3),
                      1.0f - 2.0f * (filter.q1 * filter.q1 + filter.q2 * filter.q2)) * 180.0f / PI;
        pitch = asinf(2.0f * (filter.q0 * filter.q2 - filter.q3 * filter.q1)) * 180.0f / PI;
        yaw = atan2f(2.0f * (filter.q0 * filter.q3 + filter.q1 * filter.q2),
                     1.0f - 2.0f * (filter.q2 * filter.q2 + filter.q3 * filter.q3)) * 180.0f / PI;

        // Display orientation on LCD
        char orientationBuffer[32];
        snprintf(orientationBuffer, sizeof(orientationBuffer), "R: %.1f", roll);
        lcd.printFixed(0, 0, "                     ", STYLE_NORMAL); // Clear line 1
        lcd.printFixed(0, 0, orientationBuffer, STYLE_NORMAL);

        snprintf(orientationBuffer, sizeof(orientationBuffer), "P: %.1f Y: %.1f", pitch, yaw);
        lcd.printFixed(0, 8, "                     ", STYLE_NORMAL); // Clear line 2
        lcd.printFixed(0, 8, orientationBuffer, STYLE_NORMAL);

        vTaskDelay(10 / portTICK_PERIOD_MS); // Prevent task starvation
    }
}

void newRange()
{
    DW1000Device *device = DW1000Ranging.getDistantDevice();
    if (device)
    {
        String shortAddress = String(device->getShortAddress(), HEX);
        float range = device->getRange();
        if (activeDevices.find(shortAddress) != activeDevices.end())
        {
            activeDevices[shortAddress] = {millis(), range};
            int row = std::distance(activeDevices.begin(), activeDevices.find(shortAddress)) + 2; // Adjust for orientation rows
            updateRangeOnly(row, range);
        }
        else
        {
            activeDevices[shortAddress] = {millis(), range};
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
    activeDevices[shortAddress] = {millis(), 0.0f}; // Initial range is 0.0
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
    int row = 2; // Start from the third row for UWB device information
    for (const auto &pair : activeDevices)
    {
        char buffer[32];

        // Display short address
        snprintf(buffer, sizeof(buffer), "Short: %s", pair.first.c_str());
        lcd.printFixed(0, row * 8, "                     ", STYLE_NORMAL); // Clear line
        lcd.printFixed(0, row * 8, buffer, STYLE_NORMAL);

        // Display range with fixed "m"
        snprintf(buffer, sizeof(buffer), "Range: %.2f m", pair.second.second);
        lcd.printFixed(0, row * 8 + 8, "                     ", STYLE_NORMAL); // Clear line
        lcd.printFixed(0, row * 8 + 8, buffer, STYLE_NORMAL);

        row++;
    }
}

void updateRangeOnly(int row, float range)
{
    char buffer[32];
    snprintf(buffer, sizeof(buffer), "%.2f m", range);
    lcd.printFixed(0, row * 8 + 8, "       ", STYLE_NORMAL); // Clear previous range
    lcd.printFixed(0, row * 8 + 8, buffer, STYLE_NORMAL);
}
