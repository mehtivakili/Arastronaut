#include "BMI088.h"
#include "Arduino.h"
#include <WiFiManager.h>
#include <NetworkConsole.h>

NetworkConsole networkConsole;

WiFiManager wifiManager;

/* accel object */
Bmi088Accel accel(SPI,32);
/* gyro object */
Bmi088Gyro gyro(SPI,25);

    int16_t accelX_raw, accelY_raw, accelZ_raw;
    int16_t gyroX_raw, gyroY_raw, gyroZ_raw;
// Bmi088 bmi(SPI,32,25);


// Calibration parameters
float Ta[3][3] = {{1, -0.00546066, 0.00101399}, {0, 1, 0.00141895}, {0, 0, 1}};
float Ka[3][3] = {{0.00358347, 0, 0}, {0, 0.00358133, 0}, {0, 0, 0.00359205}};
float acce_bias[3] = {-8.28051, -4.6756, -0.870355};
float Tg[3][3] = {{1, -0.00614889, -0.000546488}, {0.0102258, 1, 0.000838491}, {0.00412113, 0.0020154, 1}};
float Kg[3][3] = {{0.000531972, 0, 0}, {0, 0.000531541, 0}, {0, 0, 0.000531}};
float gyro_bias[3] = {4.53855, 4.001, -1.9779};


double Tio = 0.000;
double myArray[7];
typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

void setup() 
{
      Serial.begin(115200);

  wifiManager.begin();
    // Start WiFiManager and connect to WiFi
    wifiManager.begin();

    // Start the network console
    networkConsole.begin();
    networkConsole.println("ESP32 Network Console started.");

  int status;
  /* USB Serial to print data */
  Serial.begin(115200);
  while(!Serial) {}
  /* start the sensors */
  status = accel.begin(Bmi088Accel::RANGE_12G, Bmi088Accel::ODR_1600HZ_BW_234HZ);
  if (status < 0) {
    // Serial.println("Accel Initialization Error");
    // Serial.println(status);
    while (1) {}
  }
  status = gyro.begin(Bmi088Gyro::RANGE_1000DPS, Bmi088Gyro::ODR_2000HZ_BW_230HZ);
  if (status < 0) {
    // Serial.println("Gyro Initialization Error");
    // Serial.println(status);
    while (1) {}
  }
    // accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
    // gyro.setOdr(Bmi088Gyro::ODR_2000HZ_BW_532HZ);
    accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw );
    gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);

}

void loop() 
{
  wifiManager.handleClient();
    ArduinoOTA.handle(); // Handle OTA updates

  /* read the accel */
  accel.readSensor();
  /* read the gyro */
  gyro.readSensor();
  /* print the data */
    accel.getSensorRawValues(&accelX_raw, &accelY_raw, &accelZ_raw);
    gyro.getSensorRawValues(&gyroX_raw, &gyroY_raw, &gyroZ_raw);
  //  printf("Raw Accelerometer (X,Y,Z): %d, %d, %d\n", accelX_raw, accelY_raw, accelZ_raw);
  //  printf("Raw Gyroscope (X,Y,Z): %d, %d, %d\n", gyroX_raw, gyroY_raw, gyroZ_raw);

  // myArray[0] = accel.getAccelX_mss();
  // myArray[1] = accel.getAccelY_mss();
  // myArray[2] = accel.getAccelZ_mss();
  // myArray[3] = gyro.getGyroX_rads();
  // myArray[4] = gyro.getGyroY_rads();
  // myArray[5] = gyro.getGyroZ_rads();
  // myArray[6] = Tio;
   
  myArray[0] = Tio;
  myArray[1] = accelX_raw;
  myArray[2] = accelY_raw;
  myArray[3] = accelZ_raw;
  myArray[4] = gyroX_raw;
  myArray[5] = gyroY_raw;
  myArray[6] = gyroZ_raw;
 
 // Apply calibration to raw data
    float acce_calibrated[3];
    float gyro_calibrated[3];
    
    // for (int i = 0; i < 3; i++) {
    //     acce_calibrated[i] = Ka[i][i] * (Ta[i][i] * (accelX_raw + acce_bias[i]) + Ta[i][(i+1)%3] * (accelY_raw + acce_bias[(i+1)%3]) + Ta[i][(i+2)%3] * (accelZ_raw + acce_bias[(i+2)%3]));
    //     gyro_calibrated[i] = Kg[i][i] * (Tg[i][i] * (gyroX_raw + gyro_bias[i]) + Tg[i][(i+1)%3] * (gyroY_raw + gyro_bias[(i+1)%3]) + Tg[i][(i+2)%3] * (gyroZ_raw + gyro_bias[(i+2)%3]));
    // }

    acce_calibrated[0] = ((Ka[0][0]*Ta[0][0])+(Ka[0][1]*Ta[1][1])+(Ka[0][2]*Ta[2][2]))*(myArray[1]-acce_bias[0]);
    acce_calibrated[1] = ((Ka[1][1]*Ta[1][1])+(Ka[1][2]*Ta[2][2]))*(myArray[2]-acce_bias[1]);
    acce_calibrated[2] = ((Ka[2][2]*Ta[2][2]))*(myArray[3]-acce_bias[2]);
    gyro_calibrated[0] = ((Kg[0][0]*Tg[0][0])+(Kg[0][1]*Tg[1][1])+(Kg[0][2]*Tg[2][2]))*(myArray[4]-gyro_bias[0]);
    gyro_calibrated[1] = ((Kg[1][0]*Tg[1][0])+(Kg[1][1]*Tg[1][1])+(Kg[1][2]*Tg[2][2]))*(myArray[5]-gyro_bias[1]);
    gyro_calibrated[2] = ((Kg[2][0]*Tg[2][0])+(Kg[2][1]*Tg[2][1])+(Kg[2][2]*Tg[2][2]))*(myArray[6]-gyro_bias[2]);

    // Print calibrated data
    printf("Calibrated Accelerometer (X,Y,Z): %f,%f,%f\n",acce_calibrated[0],acce_calibrated[1],acce_calibrated[2]);
    printf("Calibrated Gyroscope (X,Y,Z): %f,%f,%f\n",gyro_calibrated[0],gyro_calibrated[1],gyro_calibrated[2]);
    networkConsole.printf("Calibrated Accelerometer (X,Y,Z): %f,%f,%f\n",acce_calibrated[0],acce_calibrated[1],acce_calibrated[2]);
    networkConsole.printf("Calibrated Gyroscope (X,Y,Z): %f,%f,%f\n",gyro_calibrated[0],gyro_calibrated[1],gyro_calibrated[2]);

  size_t size = sizeof(myArray);

  // for(int i = 0; i < 7 ; i++){
  //   binaryFloat hi;
  //   hi.floatingPoint = myArray[i];
  //   Serial.write(hi.binary,4);
  // }
  

  /* delay to help with printing */
  delay(50);
  Tio += 0.05;
}