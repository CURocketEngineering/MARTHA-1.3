#include <Arduino.h>

#ifdef SIM
  #include "simulation/Serial_Sim_LSM6DSOX.h"
  #include "simulation/Serial_Sim_LIS3MDL.h"
  #include "simulation/Serial_Sim_BMP390.h"
  #include "simulation/Serial_Sim.h"
#else
  #include "Adafruit_LSM6DSOX.h"
  #include "Adafruit_LIS3MDL.h"
  #include <Async_BMP3XX.h>
#endif

#include <Adafruit_Sensor.h>
#include <ADS1118.h>
#include "pins.h"
#include "UARTCommandHandler.h"

#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"
#include "data_handling/Telemetry.h"
#include "flash_config.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/FastLaunchDetector.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/ApogeePredictor.h"
#include "state_estimation/States.h"
#include "state_estimation/StateMachine.h"

#define SEALEVELPRESSURE_HPA (1013.25)


int last_led_toggle = 0;
int led_toggle_delay = 1000;
float loop_count = 0;
uint32_t start_time_s = 0;

Adafruit_LSM6DSOX sox;
Adafruit_LIS3MDL  mag;
ADS1118 externalAdc;


Adafruit_SPIFlash flash(&flashTransport);
DataSaverSPI dataSaver(10, &flash); // Save data every 10 ms

SensorDataHandler xAclData(ACCELEROMETER_X, &dataSaver);
SensorDataHandler yAclData(ACCELEROMETER_Y, &dataSaver);
SensorDataHandler zAclData(ACCELEROMETER_Z, &dataSaver);

SensorDataHandler xGyroData(GYROSCOPE_X, &dataSaver);
SensorDataHandler yGyroData(GYROSCOPE_Y, &dataSaver);
SensorDataHandler zGyroData(GYROSCOPE_Z, &dataSaver);

SensorDataHandler xMagData(MAGNETOMETER_X, &dataSaver);
SensorDataHandler yMagData(MAGNETOMETER_Y, &dataSaver);
SensorDataHandler zMagData(MAGNETOMETER_Z, &dataSaver);

SensorDataHandler superLoopRate(AVERAGE_CYCLE_RATE, &dataSaver);
SensorDataHandler stateChange(STATE_CHANGE, &dataSaver);
SensorDataHandler currentState(CURRENT_STATE, &dataSaver);
SensorDataHandler flightIDSaver(FLIGHT_ID, &dataSaver);
SensorDataHandler externalAdcVoltageData(EXTERNAL_ADC_VOLTAGE, &dataSaver);
float flightID;

LaunchDetector launchDetector(40, 500, 25);
FastLaunchDetector fastLaunchDetector(30, 1000);

NoiseVariances noiseVariances {0.25f, 1.0f}; // Example variances

VerticalVelocityEstimator verticalVelocityEstimator(noiseVariances);
ApogeeDetector apogeeDetector(1.0f);
StateMachine stateMachine(&dataSaver, &launchDetector, &apogeeDetector, &verticalVelocityEstimator, &fastLaunchDetector);

SensorDataHandler apogeeEstData(EST_APOGEE, &dataSaver);


HardwareSerial SUART1(PB7, PB6);

// Grab the commands
CommandLine cmdLine(&Serial);
#include "commands.h"

void setup() {

  pinMode(DEBUG_LED, OUTPUT); // LED 


  Serial.begin(115200);
  SUART1.begin(57600);
  // while (!Serial) delay(10); // Wait for Serial Monitor (Comment out if not using)



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

  if (mag.getDataRate() != LIS3MDL_DATARATE_155_HZ) {
    Serial.println("Failed to set Mag data rate");
  }

  Serial.println("Setting up external ADC...");
  externalAdc.begin_SPI(EXTERNAL_ADC_CS);
  externalAdc.setSamplingRate(externalAdc.RATE_860SPS);
  externalAdc.setInputSelected(externalAdc.AIN_0);
  externalAdc.setFullScaleRange(externalAdc.FSR_4096);

  Serial.println("Setting up data saver...");

  // Initalize data saver
  if (!dataSaver.begin()) {
    Serial.println("Failed to initialize data saver");
  }

  Serial.println("Setup complete!");

  cmdLine.addCommand("test", "t", testCommand);  
  cmdLine.addCommand("ping", "p", ping);    
  cmdLine.addCommand("clear_plm", "cplm", clearPostLaunchMode);
  cmdLine.addCommand("status", "s", printStatus);
  cmdLine.addCommand("dump", "d", dumpFlash);
  cmdLine.begin();


  // Set save speeds
  xMagData.restrictSaveSpeed(1000);
  yMagData.restrictSaveSpeed(1000);
  zMagData.restrictSaveSpeed(1000);
  superLoopRate.restrictSaveSpeed(1000);
  flightIDSaver.restrictSaveSpeed(10000);
  apogeeEstData.restrictSaveSpeed(10);
  currentState.restrictSaveSpeed(2000);
  externalAdcVoltageData.restrictSaveSpeed(10);


  // Loop start time
  start_time_s = millis() / 1000;

  // Seed the random number generator
  randomSeed(analogRead(0));

  // Set the flight ID
  flightID = random(100000, 999999);

  // Simulation stuff

  #ifdef SIM
  while (!Serial) delay(10);
  SerialSim::getInstance().begin(&Serial, &stateMachine); 
  dataSaver.clearPostLaunchMode(); // Clear plm for sim
  dataSaver.clearInternalState();
  #endif

}

void loop() {

  loop_count += 1;

  uint32_t current_time = millis();
  if (current_time - last_led_toggle > led_toggle_delay) {
    last_led_toggle = millis();
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
  }

  // Explicitly save a timestamp to ensure that all data from this loop is associated with the same timestamp and distinct from the previous loop
  dataSaver.saveTimestamp(current_time);

  flightIDSaver.addData(DataPoint(current_time, flightID));

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  sensors_event_t mag_event; 

  // Cannot use cmdLine in SIM mode b/c they use the same
  // serial port
  #ifdef SIM
  SerialSim::getInstance().update();
  #else 
  cmdLine.readInput();
  #endif

  sox.getEvent(&accel, &gyro, &temp);

  DataPoint xAclDataPoint(current_time, accel.acceleration.x);
  DataPoint yAclDataPoint(current_time, accel.acceleration.y);
  DataPoint zAclDataPoint(current_time, accel.acceleration.z);

  xAclData.addData(xAclDataPoint);
  yAclData.addData(yAclDataPoint);
  zAclData.addData(zAclDataPoint);

  AccelerationTriplet aclTriplet = {xAclDataPoint, yAclDataPoint, zAclDataPoint};

  mag.getEvent(&mag_event);

  xMagData.addData(DataPoint(current_time, mag_event.magnetic.x));
  yMagData.addData(DataPoint(current_time, mag_event.magnetic.y));
  zMagData.addData(DataPoint(current_time, mag_event.magnetic.z));

  externalAdcVoltageData.addData(DataPoint(current_time, externalAdc.getMilliVolts() / 1000.0f));

  // Will update the launch detector and apogee detector
  // Will log updates to the data saver
  // Will put the data saver in post-launch mode if the launch detector detects a launch
  // Serial.println("State machine update with alt of " + String(altDataPoint.data));
  stateMachine.update(
    aclTriplet,
    DataPoint{current_time, 0.0f}  // Placeholder altitude of 0
  );

  if (stateMachine.getState() >= STATE_ASCENT) {
    led_toggle_delay = 50;
  } else if (stateMachine.getState() == STATE_SOFT_ASCENT) {
    led_toggle_delay = 200;
  } else if (stateMachine.getState() <= STATE_ARMED){
    led_toggle_delay = 1000;
  }

  xGyroData.addData(DataPoint(current_time, gyro.gyro.x));
  yGyroData.addData(DataPoint(current_time, gyro.gyro.y));
  zGyroData.addData(DataPoint(current_time, gyro.gyro.z));

  superLoopRate.addData(DataPoint(current_time, loop_count / (millis() / 1000 - start_time_s)));
  currentState.addData(DataPoint(current_time, stateMachine.getState()));

  // Throttle to 100 Hz
  int too_fast = millis() - current_time;  // current_time was captured at the start of the loop
  if (too_fast < 10) {
    delay(10 - too_fast);
  }
}
