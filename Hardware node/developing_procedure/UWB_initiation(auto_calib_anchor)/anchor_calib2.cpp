#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

// Define to choose between MEDIAN and MEAN
#define USE_MEDIAN // Uncomment this line to use median, comment it to use mean

// ESP32_UWB pin definitions
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 5

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 5;   // spi select pin

char this_anchor_addr[] = "84:00:22:EA:82:60:3B:9C";
float this_anchor_target_distance = 3.277; // measured distance to anchor in meters

uint16_t this_anchor_Adelay = 16600; // starting value
uint16_t Adelay_delta = 50; // initial binary search step size
const int sampleSize = 50; // number of samples to collect
uint16_t adelaySamples[sampleSize]; // array to store Adelay samples
int sampleCount = 0;

const int movingAverageSize = 5; // size for moving average
float distanceSamples[movingAverageSize];
int distanceSampleCount = 0;

uint16_t calculateMedianAdelay() {
  // Sort the adelaySamples array
  for (int i = 0; i < sampleSize - 1; i++) {
    for (int j = i + 1; j < sampleSize; j++) {
      if (adelaySamples[i] > adelaySamples[j]) {
        uint16_t temp = adelaySamples[i];
        adelaySamples[i] = adelaySamples[j];
        adelaySamples[j] = temp;
      }
    }
  }
  // Return the median value
  return adelaySamples[sampleSize / 2];
}

uint16_t calculateMeanAdelay() {
  uint32_t sum = 0;
  for (int i = 0; i < sampleSize; i++) {
    sum += adelaySamples[i];
  }
  return sum / sampleSize;
}

float calculateMovingAverage(float newValue) {
  distanceSamples[distanceSampleCount % movingAverageSize] = newValue;
  distanceSampleCount++;
  float sum = 0.0;
  int count = (distanceSampleCount < movingAverageSize) ? distanceSampleCount : movingAverageSize;
  for (int i = 0; i < count; i++) {
    sum += distanceSamples[i];
  }
  return sum / count;
}

void newRange()
{
  static float last_delta = 0.0;
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);

  float dist = DW1000Ranging.getDistantDevice()->getRange();
  float smoothedDist = calculateMovingAverage(dist);

  Serial.print(",");
  Serial.print(smoothedDist, 4); // Print smoothed distance with more resolution

  float this_delta = smoothedDist - this_anchor_target_distance;  // error in measured distance

  // Print current status
  Serial.print(", Adelay = ");
  Serial.print(this_anchor_Adelay);
  Serial.print(", delta = ");
  Serial.println(this_delta, 4); // Print delta with more resolution

  // Check for stopping condition
  if (Adelay_delta < 1 && abs(this_delta) < 0.01) {
    adelaySamples[sampleCount++] = this_anchor_Adelay; // store final sample
    if (sampleCount >= sampleSize) {
      #ifdef USE_MEDIAN
        uint16_t finalAdelay = calculateMedianAdelay(); // calculate the median Adelay
      #else
        uint16_t finalAdelay = calculateMeanAdelay(); // calculate the mean Adelay
      #endif
      Serial.print("Final Adelay: ");
      Serial.println(finalAdelay);
      while(1);  // done calibrating
    }
  } else {
    if (this_delta * last_delta < 0.0) {
      Adelay_delta = Adelay_delta / 2; // sign changed, reduce step size
    }
    last_delta = this_delta;
    
    if (this_delta > 0.0) {
      this_anchor_Adelay += Adelay_delta; // new trial Adelay
    } else {
      this_anchor_Adelay -= Adelay_delta;
    }

    // Apply the new antenna delay
    DW1000.setAntennaDelay(this_anchor_Adelay);
    delay(10); // Adding a small delay to ensure the change is applied

    // Store the sample
    adelaySamples[sampleCount++] = this_anchor_Adelay;
    if (sampleCount >= sampleSize) {
      #ifdef USE_MEDIAN
        uint16_t finalAdelay = calculateMedianAdelay(); // calculate the median Adelay
      #else
        uint16_t finalAdelay = calculateMeanAdelay(); // calculate the mean Adelay
      #endif
      Serial.print("Final Adelay: ");
      Serial.println(finalAdelay);
      while(1);  // done calibrating
    }
  }

  // Reinitialize the communication if needed
  DW1000Ranging.startAsAnchor(this_anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);
  // init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); // Reset, CS, IRQ pin

  Serial.print("Starting Adelay "); Serial.println(this_anchor_Adelay);
  Serial.print("Measured distance "); Serial.println(this_anchor_target_distance);
  
  DW1000.setAntennaDelay(this_anchor_Adelay);

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  
  // start the module as anchor, don't assign random short address
  DW1000Ranging.startAsAnchor(this_anchor_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop()
{
  DW1000Ranging.loop();
}
