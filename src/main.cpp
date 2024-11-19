#include <Arduino.h>

#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LIS3MDL.h"
#include "FlashDriver.h"
#include "bmp_spi.h"
#include "pins.h"

#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"



int last_led_toggle = 0;

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  mag;
FlashDriver       flash;
BMP3_SPI          baro(PA1, PB5, PB4, PB3);

void setup() {

  pinMode(PA9, OUTPUT); //LED 


  // put your setup code here, to run once:
  Serial.begin(115200);

  // pinMode(PA4, OUTPUT); //FLASH

  FlashStatus resultFlash = flash.initFlash();
  if(resultFlash == FLASH_SUCCESS){
    Serial.println("Flash Initialized!");
  }else{
    Serial.println("FLASH Wasn't Initalized!");
  }



  Serial.println("Setting up accelerometer and gyroscope...");
  while (!sox.begin_SPI(SENSOR_LSM_CS, SENSOR_SCK , SENSOR_MISO,
                 SENSOR_MOSI)){
    Serial.println("Could not find sensor. Check wiring.");
    delay(10);
  }


  Serial.println("Setting ACL and Gyro ranges and data rates...");
  sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS );

  sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
  sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

  // If the range is not set correctly, then print a message
  if (sox.getAccelRange() != LSM6DS_ACCEL_RANGE_16_G) {
    Serial.println("Failed to set ACL range");
  }
  if (sox.getGyroRange() != LSM6DS_GYRO_RANGE_2000_DPS) {
    Serial.println("Failed to set Gyro range");
  }
  if (sox.getAccelDataRate() != LSM6DS_RATE_104_HZ) {
    Serial.println("Failed to set ACL data rate");
  }
  if (sox.getGyroDataRate() != LSM6DS_RATE_104_HZ) {
    Serial.println("Failed to set Gyro data rate");
  }

  // Setup for the magnetometer
  Serial.println("Setting up magnetometer...");
  while (!mag.begin_SPI(PA2, PA5, PA6,
                 PA7)) {
    Serial.println("Could not find sensor. Check wiring.");
    delay(10);
  }
  mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  mag.setOperationMode(LIS3MDL_SINGLEMODE);
  mag.setPerformanceMode(LIS3MDL_MEDIUMMODE);

  mag.setIntThreshold(500);
  mag.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  if (mag.getDataRate() != LIS3MDL_DATARATE_155_HZ) {
    Serial.println("Failed to set Mag data rate");
  }

    // Setup for the magnetometer
  Serial.println("Setting up barometer...");
  if(!baro.init()) {
    // May need to change chip id in driver (0x50 -> 0x60)
    Serial.println("Could not find sensor. Check wiring.");
    delay(10);
  }

}

void loop() {
  int toggle_delay = 1000;

  uint32_t current_time = millis();
  if (current_time - last_led_toggle > toggle_delay) {
    last_led_toggle = millis();
    digitalWrite(PA9, !digitalRead(PA9));
  }

  Serial.println("Hello World!");

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  float magx;
  float magy;
  float magz;
  bmp_data sensorData;

  baro.init();
  baro.getSensorData(&sensorData, true);
  mag.begin_SPI(SENSOR_LIS_CS);
  mag.readMagneticField(magx, magy, magz);
  // sox.begin_SPI(SENSOR_LSM_CS);
  // sox.getEvent(&accel, &gyro, &temp);

  float baro_tmp = sensorData.temperature;
  Serial.print("BMP390 TEMP: "); Serial.print(baro_tmp); Serial.println(" C\n");

  Serial.print("BMP390 DATA:\r\n");
  Serial.print("Pressure: "); Serial.print(sensorData.pressure); Serial.print(" Pa\t");
  Serial.print("Altitude: "); Serial.print(sensorData.altitude); Serial.print(" m\t");
  Serial.print("Temperature: "); Serial.print(sensorData.temperature); Serial.println(" C\n");

  // float acl_x = accel.acceleration.x;
  // Serial.print("ACL X: "); Serial.print(acl_x); Serial.print(" m/s^2\t");

  float gyro_x = gyro.gyro.x;
  Serial.print("GYRO X: "); Serial.print(gyro_x); Serial.print(" d/s\t");

  float mag_x = magx;
  Serial.print("MAG X: "); Serial.print(mag_x); Serial.println(" µT\n");
  
  // Serial.print("LSM6DSOX DATA:\r\n");
  // Serial.print("X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2\t");
  // Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2\t");
  // Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");
  // Serial.print("X: "); Serial.print(gyro.gyro.x); Serial.print(" d/s\t");
  // Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" d/s\t");
  // Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" d/s\n");

  // Serial.print("LIS3MDL DATA:\r\n");
  // Serial.print("X: "); Serial.print(magx); Serial.print(" µT\t");
  // Serial.print("Y: "); Serial.print(magy); Serial.print(" µT\t");
  // Serial.print("Z: "); Serial.print(magz); Serial.println(" µT\n");

  // const uint32_t testAddress = 0x00;
  // const int testLength = 255; 
  // uint8_t testData[testLength]; 
  // uint8_t readBuffer[testLength]; 
  // for (int i = 0; i < testLength; i++) {
  //   testData[i] = 0xAE; 
  // }

  // flash.eraseSector(testAddress);
  // FlashStatus writeStatus = flash.writeFlash(testAddress, testData, testLength);
  // FlashStatus readStatus = flash.readFlash(testAddress, readBuffer, testLength);

  // bool dataMatches = true;
  // for (size_t i = 0; i < testLength; i++) {
  //   if (testData[i] != readBuffer[i]) {
  //     dataMatches = false;
  //     break;
  //   }
  // }

  // if (dataMatches) {
  //   Serial.println("Flash data verification successful: Data matches!\n");
  // } else {
  //   Serial.println("Flash data verification failed: Data does not match.\n");
  // }


  delay(1000);

}


