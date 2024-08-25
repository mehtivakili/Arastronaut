/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2018 Bolder Flight Systems
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "BMI088.h"

/* BMI088 object */
Bmi088 bmi(SPI, 32, 25);
int yes = 0;
void drdy()
{
  /* read the IMU */
//   bmi.readSensor();
//   /* print the data */
//   Serial.print("bmi.getAccelX_mss()");
//   Serial.print("\t");
//   Serial.print(bmi.getAccelY_mss());
//   Serial.print("\t");
//   Serial.print(bmi.getAccelZ_mss());
//   Serial.print("\t");
//   Serial.print(bmi.getGyroX_rads());
//   Serial.print("\t");
//   Serial.print(bmi.getGyroY_rads());
//   Serial.print("\t");
//   Serial.print(bmi.getGyroZ_rads());
//   Serial.print("\t");
//   Serial.print(bmi.getTemperature_C());
//   Serial.print("\n");
    if(yes == 0){
        yes = 1;
    }
}

void setup() 
{
  int status;
  /* USB Serial to print data */
  Serial.begin(115200);
  while(!Serial) {}
  
  /* Start the sensors */
  status = bmi.begin(Bmi088::ACCEL_RANGE_3G, Bmi088::GYRO_RANGE_1000DPS, Bmi088::ODR_2000HZ);
  if (status < 0) {
    Serial.println("IMU Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  
  /* Set the ranges */
  status = bmi.setRange(Bmi088::ACCEL_RANGE_6G, Bmi088::GYRO_RANGE_500DPS);
  if (status < 0) {
    Serial.println("Failed to set ranges");
    Serial.println(status);
    while (1) {}
  }
  
  /* Set the output data rate */
  status = bmi.setOdr(Bmi088::ODR_400HZ);
  if (status < 0) {
    Serial.println("Failed to set ODR");
    Serial.println(status);
    while (1) {}
  }

  /* Map pin 26 for accelerometer data ready interrupt */
  status = bmi.mapDrdy(Bmi088::PIN_2); // Assuming PIN_2 is mapped to pin 26
  if (status < 0) {
    Serial.println("Failed to map accelerometer data ready pin");
    Serial.println(status);
    while (1) {}
  }
  
  /* Map pin 33 for gyroscope sync interrupt */
  status = bmi.mapSync(Bmi088::PIN_4); // Assuming PIN_4 is mapped to pin 33
  if (status < 0) {
    Serial.println("Failed to map gyroscope sync pin");
    Serial.println(status);
    while (1) {}
  }
  
  /* Set the data ready pin to push-pull and active high */
  status = bmi.pinModeDrdy(Bmi088::PUSH_PULL, Bmi088::ACTIVE_HIGH);
  if (status < 0) {
    Serial.println("Failed to set up data ready pin");
    Serial.println(status);
    while (1) {}
  }

  /* Attach the corresponding uC pins to interrupts */
  pinMode(26, INPUT);
  attachInterrupt(26, drdy, RISING);

//   pinMode(33, INPUT);
//   attachInterrupt(33, drdy, RISING); // Assuming you're using the same handler for both
}

void loop() 
{
    if(yes==1){
        bmi.readSensor();
        /* print the data */
        Serial.print(bmi.getAccelX_mss());
        Serial.print("\t");
        Serial.print(bmi.getAccelY_mss());
        Serial.print("\t");
        Serial.print(bmi.getAccelZ_mss());
        Serial.print("\t");
        Serial.print(bmi.getGyroX_rads());
        Serial.print("\t");
        Serial.print(bmi.getGyroY_rads());
        Serial.print("\t");
        Serial.print(bmi.getGyroZ_rads());
        Serial.print("\t");
        Serial.print(bmi.getTemperature_C());
        Serial.print("\n");
        yes = 0;
    }
}
