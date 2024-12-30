#include "uwb.h"
#include "globals.h" // Include global variables and structures
#include "DW1000.h"

// Do not redefine the variables here
// #if DEVICE_TYPE == IS_TAG
// MyLink *uwb_data = nullptr; // Define uwb_data
// #endif

void newRange() {
    uint32_t stamp = micros();
    Serial.write(USB_Packet.buffer, sizeof(USB_Packet.buffer)); // Assuming USB_Packet is defined somewhere
    update_link(uwb_data, DW1000Ranging.getDistantDevice()->getShortAddress(),
                DW1000Ranging.getDistantDevice()->getRange(),
                DW1000Ranging.getDistantDevice()->getRXPower());
}

void newDevice(DW1000Device *device) {
    add_link(uwb_data, device->getShortAddress());
    if (UWB_PRECALIBRATION) {
        applyUWBAntennaDelay(); // Apply UWB antenna delay if pre-calibration is enabled
    }
}

void inactiveDevice(DW1000Device *device) {
    delete_link(uwb_data, device->getShortAddress());
}

void setupUWB() {
    SPI.begin(18, 19, 23);
    DW1000Ranging.initCommunication(27, 5, 34);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

#if DEVICE_TYPE == IS_TAG
    DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
    uwb_data = init_link();
#else
    DW1000Ranging.startAsAnchor(anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
#endif

    if (UWB_PRECALIBRATION) {
        calibrateUWBAntennaDelay(); // Perform UWB antenna delay calibration if enabled
    }
}

// Function to perform UWB antenna delay calibration
void calibrateUWBAntennaDelay() {
    float measured_distance = initial_measured_distance; // Initial measured distance
    uint16_t Adelay_delta = 100; // Initial binary search step size
    float last_delta = 0.0;

    // Example loop to perform TWR measurements and adjust delays
    for (int i = 0; i < 1000; i++) {
        // Perform TWR measurement
        DW1000Device *device = DW1000Ranging.getDistantDevice();
        if (device) {
            measured_distance = device->getRange(); // Correct method to get distance

            // Calculate the difference between measured and actual distances
            float this_delta = measured_distance - actual_distance;

            if (Adelay_delta < 3) {
                Serial.print("Final Antenna Delay: ");
                Serial.println(tADTX); // Assuming both delays are the same for simplicity
                break; // End calibration
            }

            if (this_delta * last_delta < 0.0) Adelay_delta /= 2; // Sign changed, reduce step size
            last_delta = this_delta;

            if (this_delta > 0.0) tADTX += Adelay_delta; // Adjust delay
            else tADTX -= Adelay_delta;

            tADRX = tADTX; // Keep both delays the same for simplicity

            DW1000.setAntennaDelay(tADTX); // Update antenna delay
        }
    }

    // Print the calibrated antenna delays
    Serial.print("Calibrated Transmit Antenna Delay: ");
    Serial.println(tADTX);
    Serial.print("Calibrated Receive Antenna Delay: ");
    Serial.println(tADRX);
}

// Function to apply UWB antenna delays to a device
void applyUWBAntennaDelay() {
    // Set antenna delay using DW1000 library function
    DW1000.setAntennaDelay(tADTX); // Set TX and RX antenna delay
}
