#include <Arduino.h>

#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LIS3MDL.h"
#include "FlashDriver.h"
#include <Adafruit_Sensor.h>
#include <Async_BMP3XX.h>
#include "pins.h"
#include "UARTCommandHandler.h"

#include "data_handling/SensorDataHandler.h"
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataNames.h"
#include "flash_config.h"
#include "state_estimation/LaunchPredictor.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/States.h"
#include "state_estimation/StateMachine.h"

#define SEALEVELPRESSURE_HPA (1013.25)


int last_led_toggle = 0;
int led_toggle_delay = 1000;
float loop_count = 0;
uint32_t start_time_s = 0;

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
DataPoint altDataPoint;

SensorDataHandler xMagData(MAGNETOMETER_X, &dataSaver);
SensorDataHandler yMagData(MAGNETOMETER_Y, &dataSaver);
SensorDataHandler zMagData(MAGNETOMETER_Z, &dataSaver);

SensorDataHandler superLoopRate(AVERAGE_CYCLE_RATE, &dataSaver);
SensorDataHandler stateChange(STATE_CHANGE, &dataSaver);
SensorDataHandler flightIDSaver(FLIGHT_ID, &dataSaver);
float flightID;

LaunchPredictor launchPredictor(40, 500, 25);
ApogeeDetector apogeeDetector(0.25f, 1.0f, 2.0f);
StateMachine stateMachine(&dataSaver, &launchPredictor, &apogeeDetector);

CommandLine cmdLine(&Serial);

HardwareSerial SUART1(PB7, PB6);

void testCommand(queue<string> arguments, string& response);
void ping(queue<string> arguments, string& response);
void dumpFlash(queue<string> arguments, string& response);
void clearPostLaunchMode(queue<string> arguments, string& response);
void printStatus(queue<string> arguments, string& response);
float parseNextFloat(String& data);

void setup() {

  pinMode(DEBUG_LED, OUTPUT); // LED 


  Serial.begin(115200);
  // while (!Serial) delay(10); // Wait for Serial Monitor (Comment out if not using)

  SUART1.begin(115200);
  while (!SUART1) delay(10);
  SUART1.write('S'); SUART1.write('T'); SUART1.write('A'); SUART1.write('R'); SUART1.write('T');



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
  tempData.restrictSaveSpeed(1000);
  pressureData.restrictSaveSpeed(1000);
  xMagData.restrictSaveSpeed(1000);
  yMagData.restrictSaveSpeed(1000);
  zMagData.restrictSaveSpeed(1000);
  superLoopRate.restrictSaveSpeed(1000);
  altitudeData.restrictSaveSpeed(10); // Save altitude every 10 ms (100hz)
  flightIDSaver.restrictSaveSpeed(10000);


  // Loop start time
  start_time_s = millis() / 1000;

  // Seed the random number generator
  randomSeed(analogRead(0));

  // Set the flight ID
  flightID = random(100000, 999999);

}

void loop() {

  cmdLine.readInput();

  loop_count += 1;

  uint32_t current_time = millis();
  if (current_time - last_led_toggle > led_toggle_delay) {
    last_led_toggle = millis();
    digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
  }

  // Explicitly save a timestamp to ensure that all data from this loop is associated with the same timestamp and distinct from the previous loop
  dataSaver.saveTimestamp(current_time, TIMESTAMP);

  flightIDSaver.addData(DataPoint(current_time, flightID));

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t mag_event; 
  float temp;
  float pres;
  float alt;

  if (SUART1.available()) {

    String incomingData = SUART1.readStringUntil('\n');
    int idx = 0;

    float timestamp_temp = parseNextFloat(incomingData);
            
    accel.acceleration.x = parseNextFloat(incomingData);
    accel.acceleration.y = parseNextFloat(incomingData);
    accel.acceleration.z = parseNextFloat(incomingData);

    gyro.gyro.x = parseNextFloat(incomingData);
    gyro.gyro.y = parseNextFloat(incomingData);
    gyro.gyro.z = parseNextFloat(incomingData);

    mag_event.magnetic.x = parseNextFloat(incomingData);
    mag_event.magnetic.y = parseNextFloat(incomingData);
    mag_event.magnetic.z = parseNextFloat(incomingData);

    alt = parseNextFloat(incomingData);
    pres = parseNextFloat(incomingData);
    temp = incomingData.toFloat(); // No comma to split after the last field
    

    xMagData.addData(DataPoint(current_time, mag_event.magnetic.x));
    yMagData.addData(DataPoint(current_time, mag_event.magnetic.y));
    zMagData.addData(DataPoint(current_time, mag_event.magnetic.z));


    DataPoint xAclDataPoint(current_time, accel.acceleration.x);
    DataPoint yAclDataPoint(current_time, accel.acceleration.y);
    DataPoint zAclDataPoint(current_time, accel.acceleration.z);

    xAclData.addData(xAclDataPoint);
    yAclData.addData(yAclDataPoint);
    zAclData.addData(zAclDataPoint);

    
    tempData.addData(DataPoint(current_time, temp));
    pressureData.addData(DataPoint(current_time, pres));
    altDataPoint.data = alt;
    altDataPoint.timestamp_ms = current_time;
    altitudeData.addData(altDataPoint);
      
      // Immediately start the next conversion


    // Will update the launch predictor and apogee detector
    // Will log updates to the data saver
    // Will put the data saver in post-launch mode if the launch predictor detects a launch
    // Serial.println("State machine update with alt of " + String(altDataPoint.data));
    stateMachine.update(
      xAclDataPoint,
      yAclDataPoint,
      zAclDataPoint,
      altDataPoint
    );

    if (stateMachine.getState() > STATE_ASCENT) {
      led_toggle_delay = 50;
    } else if (stateMachine.getState() > STATE_ARMED || dataSaver.quickGetPostLaunchMode()) {
      led_toggle_delay = 100;
    }

    xGyroData.addData(DataPoint(current_time, gyro.gyro.x));
    yGyroData.addData(DataPoint(current_time, gyro.gyro.y));
    zGyroData.addData(DataPoint(current_time, gyro.gyro.z));

    superLoopRate.addData(DataPoint(current_time, loop_count / (millis() / 1000 - start_time_s)));

    SUART1.write('A');  // Send 'A' as acknowledgment
  }

  // Throttle to 100 Hz
  int too_fast = millis() - current_time;  // current_time was captured at the start of the loop
  if (too_fast < 10) {
    delay(10 - too_fast);
  }

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

void clearPostLaunchMode(queue<string> arguments, string& response) {
    dataSaver.clearPostLaunchMode();
    launchPredictor.reset();
    cmdLine.println("Cleared post launch mode, reboot the device to complete the reset.");
}

std::string floatToString(float value, int precision = 2) {
    char buffer[20];
    dtostrf(value, 0, precision, buffer);
    return std::string(buffer);
}

void dumpFlash(std::queue<std::string> arguments, std::string& response) {
    // check for -a in arg
    if (arguments.empty()) {
        dataSaver.dumpData(Serial, false);
        return;
    } else if (arguments.front() == "-a") {
        arguments.pop();
        dataSaver.dumpData(Serial, true);
        return;
    } else {
      cmdLine.println("Invalid argument. Use -a to ignore empty pages.");
    }
}

void printStatus(std::queue<std::string> arguments, std::string& response) {
    cmdLine.println("--Launch Predictor--");
    cmdLine.print("Launched: ");
    cmdLine.println(std::to_string(launchPredictor.isLaunched()));
    cmdLine.print("Launched Time: ");
    cmdLine.println(floatToString(launchPredictor.getLaunchedTime()));
    cmdLine.print("Median Acceleration Squared: ");
    cmdLine.println(floatToString(launchPredictor.getMedianAccelerationSquared()));

    cmdLine.println("");
    cmdLine.println("--Apogee Detector--");
    cmdLine.print("Apogee Detected: ");
    cmdLine.println(std::to_string(apogeeDetector.isApogeeDetected()));
    cmdLine.print("Estimated Altitude: ");
    cmdLine.println(floatToString(apogeeDetector.getEstimatedAltitude()));
    cmdLine.print("Estimated Velocity: ");
    cmdLine.println(floatToString(apogeeDetector.getEstimatedVelocity()));
    cmdLine.print("Inertial Vertical Acceleration: ");
    cmdLine.println(floatToString(apogeeDetector.getInertialVerticalAcceleration()));
    cmdLine.print("Vertical Axis: ");
    cmdLine.println(std::to_string(apogeeDetector.getVerticalAxis()));
    cmdLine.print("Vertical Direction: ");
    cmdLine.println(std::to_string(apogeeDetector.getVerticalDirection()));
    cmdLine.print("Apogee Altitude: ");
    cmdLine.println(floatToString(apogeeDetector.getApogee().data));

    cmdLine.println("");
    cmdLine.println("--Data Saver--");
    cmdLine.print("Post Launch Mode: ");
    cmdLine.println(std::to_string(dataSaver.quickGetPostLaunchMode()));
    cmdLine.print("Rebooted in Post Launch Mode (won't save): ");
    cmdLine.println(std::to_string(dataSaver.getRebootedInPostLaunchMode()));
    cmdLine.print("Last Timestamp: ");
    cmdLine.println(std::to_string(dataSaver.getLastTimestamp()));
    cmdLine.print("Last Data Point Value: ");
    cmdLine.println(floatToString(dataSaver.getLastDataPoint().data));
    cmdLine.print("Super loop average hz: ");
    cmdLine.println(floatToString(loop_count / (millis() / 1000 - start_time_s)));

    cmdLine.println("");
    cmdLine.println("--Flash--");
    cmdLine.print("Stopped writing b/c wrapped around to launch address: ");
    cmdLine.println(std::to_string(dataSaver.getIsChipFullDueToPostLaunchProtection()));
    cmdLine.print("Launch Write Address: ");
    cmdLine.println(std::to_string(dataSaver.getLaunchWriteAddress()));
    cmdLine.print("Next Write Address: ");
    cmdLine.println(std::to_string(dataSaver.getNextWriteAddress()));
    cmdLine.print("Buffer Index: ");
    cmdLine.println(std::to_string(dataSaver.getBufferIndex()));
    cmdLine.print("Buffer Flushes: ");
    cmdLine.println(std::to_string(dataSaver.getBufferFlushes()));

    cmdLine.println("");
    cmdLine.println("--Sensors--");
    cmdLine.print("Accelerometer X: ");
    cmdLine.println(floatToString(xAclData.getLastDataPointSaved().data));
    cmdLine.print("Accelerometer Y: ");
    cmdLine.println(floatToString(yAclData.getLastDataPointSaved().data));
    cmdLine.print("Accelerometer Z: ");
    cmdLine.println(floatToString(zAclData.getLastDataPointSaved().data));
    cmdLine.print("Gyroscope X: ");
    cmdLine.println(floatToString(xGyroData.getLastDataPointSaved().data));
    cmdLine.print("Gyroscope Y: ");
    cmdLine.println(floatToString(yGyroData.getLastDataPointSaved().data));
    cmdLine.print("Gyroscope Z: ");
    cmdLine.println(floatToString(zGyroData.getLastDataPointSaved().data));
    cmdLine.print("Temperature: ");
    cmdLine.println(floatToString(tempData.getLastDataPointSaved().data));
    cmdLine.print("Pressure: ");
    cmdLine.println(floatToString(pressureData.getLastDataPointSaved().data));
    cmdLine.print("Altitude: ");
    cmdLine.println(floatToString(altitudeData.getLastDataPointSaved().data));
    cmdLine.print("Magnetometer X: ");
    cmdLine.println(floatToString(xMagData.getLastDataPointSaved().data));
    cmdLine.print("Magnetometer Y: ");
    cmdLine.println(floatToString(yMagData.getLastDataPointSaved().data));
    cmdLine.print("Magnetometer Z: ");
    cmdLine.println(floatToString(zMagData.getLastDataPointSaved().data));
}

float parseNextFloat(String& data) {
  float value = data.substring(0, data.indexOf(',')).toFloat();  // Extract value as float
  data = data.substring(data.indexOf(',') + 1);  // Update data by removing the parsed value
  return value;
}
