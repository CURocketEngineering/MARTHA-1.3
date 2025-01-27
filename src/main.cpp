#include <Arduino.h>

#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LIS3MDL.h"
#include "FlashDriver.h"
#include "CC1125.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include "pins.h"
#include "UARTCommandHandler.h"

#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"
#include "flash_config.h"
#include "data_handling/LaunchPredictor.h"

#define SEALEVELPRESSURE_HPA (1013.25)


int last_led_toggle = 0;
int led_toggle_delay = 1000;
float loop_count = 0;

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  mag;
Adafruit_BMP3XX   bmp;

Adafruit_SPIFlash flash(&flashTransport);
DataSaverSPI dataSaver(10, &flash); // Save data every 10 ms

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

LaunchPredictor launchPredictor(30, 1000, 40);

CommandLine cmdLine(&Serial);

#ifdef MASON_MARTHA_PCB
  CC1125 rf(CC1125_RESET, CC1125_CS, &sox, &mag, &bmp);
#endif

void testCommand(queue<string> arguments, string& response);
void ping(queue<string> arguments, string& response);
void dumpFlash(queue<string> arguments, string& response);
void groundStation(queue<string> arguments, string& response)

void setup() {

  pinMode(DEBUG_LED, OUTPUT); // LED 


  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial Monitor (Comment out if not using)


  Serial.println("Setting up accelerometer and gyroscope...");
  while (!sox.begin_SPI(SENSOR_LSM_CS)){
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
  while (!mag.begin_SPI(SENSOR_LIS_CS)) {
    Serial.println("Could not find sensor. Check wiring.");
    delay(10);
  }
  mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
  mag.setRange(LIS3MDL_RANGE_4_GAUSS);
  mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  mag.setPerformanceMode(LIS3MDL_MEDIUMMODE);

  mag.setIntThreshold(500);
  mag.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!

  if (mag.getDataRate() != LIS3MDL_DATARATE_155_HZ) {
    Serial.println("Failed to set Mag data rate");
  }

  while (! bmp.begin_SPI(SENSOR_BARO_CS)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    delay(10);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("Setting up data saver...");

  // Initalize data saver
  if (!dataSaver.begin()) {
    Serial.println("Failed to initialize data saver");
  }

  Serial.println("Setup complete!");

  cmdLine.addCommand("test", "t", testCommand);  
  cmdLine.addCommand("ping", "p", ping);  
  cmdLine.addCommand("telemtry", "gs", groundStation) ; 
  cmdLine.begin();


}

void loop() {

  cmdLine.readInput();

  loop_count += 1;

  uint32_t current_time = millis();
  if (current_time - last_led_toggle > led_toggle_delay) {
    last_led_toggle = millis();
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
  }

  // Serial.write("Reading sensors...\n");

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag_event; 

  mag.getEvent(&mag_event);

  xMagData.addData(DataPoint(current_time, mag_event.magnetic.x));
  yMagData.addData(DataPoint(current_time, mag_event.magnetic.y));
  zMagData.addData(DataPoint(current_time, mag_event.magnetic.z));

  sox.getEvent(&accel, &gyro, &temp);

  // Serial.write("ACL X: ");
  // Serial.println(accel.acceleration.x);
  // Serial.write("GYRO X: ");
  // Serial.println(gyro.gyro.x);
  // Serial.write("TEMP: ");
  // Serial.println(temp.temperature);
  // Serial.write("MAG X: ");
  // Serial.println(mag_event.magnetic.x);
  // Serial.write("Altitude: ");
  // Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));

  xAclData.addData(DataPoint(current_time, accel.acceleration.x));
  yAclData.addData(DataPoint(current_time, accel.acceleration.y));
  zAclData.addData(DataPoint(current_time, accel.acceleration.z));

  launchPredictor.update(DataPoint(current_time, accel.acceleration.x),
                         DataPoint(current_time, accel.acceleration.y),
                         DataPoint(current_time, accel.acceleration.z));

  // Blink fast if in post launch mode (DO NOT LAUNCH)
  if (dataSaver.quickGetPostLaunchMode()) {
    led_toggle_delay = 100;
  } else {
    // If launch detected, put the dataSaver into post launch mode
    // i.e. all the data written from this point on is sacred and will not be overwritten
    if (launchPredictor.isLaunched()){
      Serial.println("Launch detected!");
      dataSaver.launchDetected(launchPredictor.getLaunchedTime());
    }
  }

  // Serial.println(launchPredictor.getMedianAccelerationSquared());

  xGyroData.addData(DataPoint(current_time, gyro.gyro.x));
  yGyroData.addData(DataPoint(current_time, gyro.gyro.y));
  zGyroData.addData(DataPoint(current_time, gyro.gyro.z));

  tempData.addData(DataPoint(current_time, temp.temperature));

  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  altitudeData.addData(DataPoint(current_time, bmp.readAltitude(SEALEVELPRESSURE_HPA)));
  pressureData.addData(DataPoint(current_time, bmp.pressure));

  // Serial.print("Loop time: ");
  // Serial.println((millis() - current_time));

}


void testCommand(std::queue<std::string> arguments, std::string& response) {
    cmdLine.println("Test command executed.");
    
    // Check if there are any arguments
    if (arguments.empty()) {
        cmdLine.println("No arguments provided.");
        response = "Test command executed. Arguments: None";
    } else {
        cmdLine.println("Arguments received:");
        response = "Test command executed. Arguments: ";
        
        // Process each argument
        while (!arguments.empty()) {
            std::string argument = arguments.front();
            arguments.pop();
            
            // Print each argument to the UART
            cmdLine.println(" - " + argument);
            
            // Append the argument to the response
            response += argument + " ";
        }
    }
}


void ping(queue<string> arguments, string& response) {
    cmdLine.println("Pinged the microntroller ");
}

#ifdef MASON_MARTHA_PCB
void groundStation(queue<string> arguments, string& response)
{
  CC1125Status status;
  status = rf.init();
  if(status != CC1125_SUCCESS)
  {
    Serial.println("Failed to initialize CC1125");
  }
  else
  {
    Serial.println("Initialize CC1125");
    rf.telemetryGroundStation();
  }

  
}
#endif
