#include <Arduino.h>

#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LIS3MDL.h"
#include "FlashDriver.h"
#include "bmp_spi.h"

#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"





Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  mag;
FlashDriver       flash;
BMP3_SPI          baro(PA1, PB5, PB4, PB3);

void setup() {


  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(PA4, OUTPUT); //FLASH

  FlashStatus resultFlash = flash.initFlash();
  if(resultFlash == FLASH_SUCCESS){
    Serial.println("Flash Initialized!");
  }else{
    Serial.println("FLASH Wasn't Initalized!");
  }



  Serial.println("Setting up accelerometer and gyroscope...");
  while (!sox.begin_SPI(PA0, PB3, PB4,
                 PB5)){
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
  while (!mag.begin_SPI(PA3, PB3, PB4,
                 PB5)) {
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

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  float magx;
  float magy;
  float magz;
  bmp_data sensorData;

  baro.init();
  baro.getSensorData(&sensorData, true);
  mag.begin_SPI(PA3);
  mag.readMagneticField(magx, magy, magz);
  sox.begin_SPI(PA0);
  sox.getEvent(&accel, &gyro, &temp);


  Serial.print("BMP390 DATA:\r\n");
  Serial.print("Pressure: "); Serial.print(sensorData.pressure); Serial.print(" Pa\t");
  Serial.print("Altitude: "); Serial.print(sensorData.altitude); Serial.print(" m\t");
  Serial.print("Temperature: "); Serial.print(sensorData.temperature); Serial.println(" C\n");

  Serial.print("LSM6DSOX DATA:\r\n");
  Serial.print("X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2\t");
  Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2\t");
  Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");
  Serial.print("X: "); Serial.print(gyro.gyro.x); Serial.print(" d/s\t");
  Serial.print("Y: "); Serial.print(gyro.gyro.y); Serial.print(" d/s\t");
  Serial.print("Z: "); Serial.print(gyro.gyro.z); Serial.println(" d/s\n");

  Serial.print("LIS3MDL DATA:\r\n");
  Serial.print("X: "); Serial.print(magx); Serial.print(" µT\t");
  Serial.print("Y: "); Serial.print(magy); Serial.print(" µT\t");
  Serial.print("Z: "); Serial.print(magz); Serial.println(" µT\n");

  const uint32_t testAddress = 0x00;
  const int testLength = 255; 
  uint8_t testData[testLength]; 
  uint8_t readBuffer[testLength]; 
  for (int i = 0; i < testLength; i++) {
    testData[i] = 0xAE; 
  }

  flash.eraseSector(testAddress);
  FlashStatus writeStatus = flash.writeFlash(testAddress, testData, testLength);
  FlashStatus readStatus = flash.readFlash(testAddress, readBuffer, testLength);

  bool dataMatches = true;
  for (size_t i = 0; i < testLength; i++) {
    if (testData[i] != readBuffer[i]) {
      dataMatches = false;
      break;
    }
  }

  if (dataMatches) {
    Serial.println("Flash data verification successful: Data matches!\n");
  } else {
    Serial.println("Flash data verification failed: Data does not match.\n");
  }


  delay(1000);

}


