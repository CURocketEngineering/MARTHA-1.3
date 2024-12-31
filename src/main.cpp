#include <Arduino.h>

#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LIS3MDL.h"
#include "FlashDriver.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "pins.h"

#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"

#define SEALEVELPRESSURE_HPA (1013.25)


int last_led_toggle = 0;

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  mag;
FlashDriver       flash(SENSOR_MOSI, SENSOR_MISO, SENSOR_SCK, FLASH_CS);
Adafruit_BMP3XX   bmp;

DataSaverSPI dataSaver(100, SENSOR_MOSI, SENSOR_MISO, SENSOR_SCK, FLASH_CS);

SensorDataHandler xAclData(ACCELEROMETER_X, &dataSaver);
SensorDataHandler yAclData(ACCELEROMETER_Y, &dataSaver);
SensorDataHandler zAclData(ACCELEROMETER_Z, &dataSaver);

SensorDataHandler xGyroData(GYROSCOPE_X, &dataSaver);
SensorDataHandler yGyroData(GYROSCOPE_Y, &dataSaver);
SensorDataHandler zGyroData(GYROSCOPE_Z, &dataSaver);

SensorDataHandler tempData(TEMPERATURE, &dataSaver);
SensorDataHandler pressureData(PRESSURE, &dataSaver);
SensorDataHandler altitudeData(ALTITUDE, &dataSaver);

SensorDataHandler xMagData(MAGNETOMETER_X, &dataSaver);
SensorDataHandler yMagData(MAGNETOMETER_Y, &dataSaver);
SensorDataHandler zMagData(MAGNETOMETER_Z, &dataSaver);

void setup() {

  pinMode(PA9, OUTPUT); // LED 


  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(PA4, OUTPUT); // FLASH

  FlashStatus resultFlash = flash.initFlash();
  if(resultFlash == FLASH_SUCCESS){
    Serial.println("Flash Initialized!");
  } else{
    Serial.println("Flash Wasn't Initalized!");
  }



  Serial.println("Setting up accelerometer and gyroscope...");
  while (!sox.begin_SPI(SENSOR_LSM_CS, SENSOR_SCK , SENSOR_MISO,
                 SENSOR_MOSI)){
    Serial.println("Could not find LSM6DSOX. Check wiring.");
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
  while (!mag.begin_SPI(SENSOR_LIS_CS, SENSOR_SCK, SENSOR_MISO,
                 SENSOR_MOSI)) {
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

  if (! bmp.begin_SPI(SENSOR_BARO_CS, SENSOR_SCK, SENSOR_MISO, SENSOR_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void loop() {
  int toggle_delay = 1000;

  uint32_t current_time = millis();
  if (current_time - last_led_toggle > toggle_delay) {
    last_led_toggle = millis();
    digitalWrite(PA9, !digitalRead(PA9));
  }

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag_event; 

  mag.getEvent(&mag_event);

  xMagData.addData(DataPoint(mag_event.magnetic.x, millis()));
  yMagData.addData(DataPoint(mag_event.magnetic.y, millis()));
  zMagData.addData(DataPoint(mag_event.magnetic.z, millis()));

  sox.getEvent(&accel, &gyro, &temp);

  xAclData.addData(DataPoint(accel.acceleration.x, millis()));
  yAclData.addData(DataPoint(accel.acceleration.y, millis()));
  zAclData.addData(DataPoint(accel.acceleration.z, millis()));

  xGyroData.addData(DataPoint(gyro.gyro.x, millis()));
  yGyroData.addData(DataPoint(gyro.gyro.y, millis()));
  zGyroData.addData(DataPoint(gyro.gyro.z, millis()));

  tempData.addData(DataPoint(temp.temperature, millis()));

  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  altitudeData.addData(DataPoint(bmp.readAltitude(SEALEVELPRESSURE_HPA), millis()));
  pressureData.addData(DataPoint(bmp.pressure, millis()));

  // Serial.println("Alive");

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

}


