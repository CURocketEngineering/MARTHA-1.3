#include <Arduino.h>

#include "bmp_spi.h"
#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LIS3MDL.h"
#include "FlashDriver.h"

#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"

Adafruit_LSM6DSOX sox;
FlashDriver flash;

void setup() {


  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PA4, OUTPUT);
  FlashStatus result = flash.initFlash();
  
  if(result == FLASH_SUCCESS){
    Serial.println("Flash Initialized!");
  } else if(result == FLASH_INVALID){
    Serial.println("SPI Wasn't Initalized!");
  }else{
    Serial.println("FLASH Wasn't Initalized!");
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
}

void loop() {

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sox.getEvent(&accel, &gyro, &temp);

  
  Serial.print("X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2\t");
  Serial.print("Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2\t");
  Serial.print("Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");



}


