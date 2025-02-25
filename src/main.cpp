#include <Arduino.h>

#include "Adafruit_LSM6DSOX.h"
#include "Adafruit_LIS3MDL.h"
#include "Adafruit_BMP3XX.h"
// #include "FlashDriver.h"
#include <Adafruit_Sensor.h>
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

#include "communication/CommandHandler.h"
#include "communication/CoreCommands.h"
#include "communication/FlashCommands.h"

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

SensorDataHandler xMagData(MAGNETOMETER_X, &dataSaver);
SensorDataHandler yMagData(MAGNETOMETER_Y, &dataSaver);
SensorDataHandler zMagData(MAGNETOMETER_Z, &dataSaver);

SensorDataHandler superLoopRate(AVERAGE_CYCLE_RATE, &dataSaver);

LaunchPredictor launchPredictor(30, 1000, 40);

// Create command handler and flash driver
std::shared_ptr<CommandHandler> cmdHandler = std::make_shared<CommandHandler>();
// FlashDriver flashDriver(&flash);

void setup() {
    pinMode(DEBUG_LED, OUTPUT); // LED


    Serial.begin(115200);
    while (!Serial) delay(10); // Wait for Serial Monitor

    // Initialize core commands with versions
    CoreCommands::bindToHandler(cmdHandler);
    auto core = std::make_shared<CoreCommands>();
    core->addVersion("Board", "MARTHA 1.3");
    core->addVersion("CommandHandler", "1.0.0");
    core->addVersion("FlashCommands", "1.0.0");

    // Initialize flash commands
    auto flashCommands = std::make_shared<FlashCommands>(&flashDriver);
    cmdHandler->bindCommand(FlashCommands::getFlashDumpCommand(),
        std::bind(&FlashCommands::handleFlashDump, flashCommands, std::placeholders::_1));

    Serial.println("Setting up accelerometer and gyroscope...");
    while (!sox.begin_SPI(SENSOR_LSM_CS)){
        Serial.println("Could not find LSM6DSOX. Check wiring.");
        delay(10);
    }

    Serial.println("Setting ACL and Gyro ranges and data rates...");
    sox.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    sox.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
    sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
    sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

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

    Serial.println("Setting up magnetometer...");
    while (!mag.begin_SPI(SENSOR_LIS_CS)) {
        Serial.println("Could not find sensor. Check wiring.");
        delay(10);
    }
    mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setPerformanceMode(LIS3MDL_MEDIUMMODE);

  // mag.setIntThreshold(500);
  // mag.configInterrupt(false, false, true, // enable z axis
  //                         true, // polarity
  //                         false, // don't latch
  //                         true); // enabled!

    if (mag.getDataRate() != LIS3MDL_DATARATE_155_HZ) {
        Serial.println("Failed to set Mag data rate");
    }

    while (!bmp.begin_SPI(SENSOR_BARO_CS)) {
        Serial.println("Could not find a valid BMP3 sensor, check wiring!");
        delay(10);
    }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);

    Serial.println("Setting up data saver...");
    if (!dataSaver.begin()) {
        Serial.println("Failed to initialize data saver");
    }

    // Set save speeds
  tempData.restrictSaveSpeed(1000);
  pressureData.restrictSaveSpeed(1000);
  xMagData.restrictSaveSpeed(1000);
  yMagData.restrictSaveSpeed(1000);
  zMagData.restrictSaveSpeed(1000);
  superLoopRate.restrictSaveSpeed(1000);


  // Loop start time
  start_time_s = millis() / 1000;

    Serial.println("Setup complete!");
}

void loop() {
    // Process any incoming command bytes
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        if (cmdHandler->processCommandByte(byte)) {
            // Send response
            const auto& response = cmdHandler->getResponse();
            Serial.write(response.data(), response.size());
        }
    }

    loop_count += 1;

    uint32_t current_time = millis();
    if (current_time - last_led_toggle > led_toggle_delay) {
        last_led_toggle = millis();
        digitalWrite(DEBUG_LED, !digitalRead(DEBUG_LED));
    }

    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    sensors_event_t mag_event;

    mag.getEvent(&mag_event);

    xMagData.addData(DataPoint(current_time, mag_event.magnetic.x));
    yMagData.addData(DataPoint(current_time, mag_event.magnetic.y));
    zMagData.addData(DataPoint(current_time, mag_event.magnetic.z));

    sox.getEvent(&accel, &gyro, &temp);

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
        if (launchPredictor.isLaunched()) {
            Serial.println("Launch detected!");
            dataSaver.launchDetected(launchPredictor.getLaunchedTime());
        }
    }

    xGyroData.addData(DataPoint(current_time, gyro.gyro.x));
    yGyroData.addData(DataPoint(current_time, gyro.gyro.y));
    zGyroData.addData(DataPoint(current_time, gyro.gyro.z));

    tempData.addData(DataPoint(current_time, temp.temperature));

    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }

  altitudeData.addData(DataPoint(current_time, bmp.readAltitude(SEALEVELPRESSURE_HPA)));
  pressureData.addData(DataPoint(current_time, bmp.pressure));

  superLoopRate.addData(DataPoint(current_time, loop_count / (millis() / 1000 - start_time_s)));

  // print in hz the loop rate
  // Serial.println(loop_count / (millis() / 1000 - start_time_s));

}


// void testCommand(std::queue<std::string> arguments, std::string& response) {
//     cmdLine.println("Test command executed.");
    
//     // Check if there are any arguments
//     if (arguments.empty()) {
//         cmdLine.println("No arguments provided.");
//         response = "Test command executed. Arguments: None";
//     } else {
//         cmdLine.println("Arguments received:");
//         response = "Test command executed. Arguments: ";
        
//         // Process each argument
//         while (!arguments.empty()) {
//             std::string argument = arguments.front();
//             arguments.pop();
            
//             // Print each argument to the UART
//             cmdLine.println(" - " + argument);
            
//             // Append the argument to the response
//             response += argument + " ";
//         }
//     }
// }


// void ping(queue<string> arguments, string& response) {
//     cmdLine.println("Pinged the microntroller ");
// }

// void clearPostLaunchMode(queue<string> arguments, string& response) {
//     dataSaver.clearPostLaunchMode();
//     launchPredictor.reset();
//     cmdLine.println("Cleared post launch mode, reboot the device to complete the reset.");
// }

// std::string floatToString(float value, int precision = 2) {
//     char buffer[20];
//     dtostrf(value, 0, precision, buffer);
//     return std::string(buffer);
// }

// void dumpFlash(std::queue<std::string> arguments, std::string& response) {
//     // check for -a in arg
//     if (arguments.empty()) {
//         dataSaver.dumpData(Serial, false);
//         return;
//     } else if (arguments.front() == "-a") {
//         arguments.pop();
//         dataSaver.dumpData(Serial, true);
//         return;
//     } else {
//       cmdLine.println("Invalid argument. Use -a to ignore empty pages.");
//     }
// }

// void printStatus(std::queue<std::string> arguments, std::string& response) {
//     cmdLine.println("--Launch Predictor--");
//     cmdLine.print("Launched: ");
//     cmdLine.println(std::to_string(launchPredictor.isLaunched()));
//     cmdLine.print("Launched Time: ");
//     cmdLine.println(floatToString(launchPredictor.getLaunchedTime()));
//     cmdLine.print("Median Acceleration Squared: ");
//     cmdLine.println(floatToString(launchPredictor.getMedianAccelerationSquared()));

//     cmdLine.println("");
//     cmdLine.println("--Apogee Detector--");
//     cmdLine.print("Apogee Detected: ");
//     cmdLine.println(std::to_string(apogeeDetector.isApogeeDetected()));
//     cmdLine.print("Estimated Altitude: ");
//     cmdLine.println(floatToString(apogeeDetector.getEstimatedAltitude()));
//     cmdLine.print("Estimated Velocity: ");
//     cmdLine.println(floatToString(apogeeDetector.getEstimatedVelocity()));
//     cmdLine.print("Inertial Vertical Acceleration: ");
//     cmdLine.println(floatToString(apogeeDetector.getInertialVerticalAcceleration()));
//     cmdLine.print("Vertical Axis: ");
//     cmdLine.println(std::to_string(apogeeDetector.getVerticalAxis()));
//     cmdLine.print("Vertical Direction: ");
//     cmdLine.println(std::to_string(apogeeDetector.getVerticalDirection()));
//     cmdLine.print("Apogee Altitude: ");
//     cmdLine.println(floatToString(apogeeDetector.getApogee().data));

//     cmdLine.println("");
//     cmdLine.println("--Data Saver--");
//     cmdLine.print("Post Launch Mode: ");
//     cmdLine.println(std::to_string(dataSaver.quickGetPostLaunchMode()));
//     cmdLine.print("Rebooted in Post Launch Mode (won't save): ");
//     cmdLine.println(std::to_string(dataSaver.getRebootedInPostLaunchMode()));
//     cmdLine.print("Last Timestamp: ");
//     cmdLine.println(std::to_string(dataSaver.getLastTimestamp()));
//     cmdLine.print("Last Data Point Value: ");
//     cmdLine.println(floatToString(dataSaver.getLastDataPoint().data));
//     cmdLine.print("Super loop average hz: ");
//     cmdLine.println(floatToString(loop_count / (millis() / 1000 - start_time_s)));

//     cmdLine.println("");
//     cmdLine.println("--Flash--");
//     cmdLine.print("Stopped writing b/c wrapped around to launch address: ");
//     cmdLine.println(std::to_string(dataSaver.getIsChipFullDueToPostLaunchProtection()));
//     cmdLine.print("Launch Write Address: ");
//     cmdLine.println(std::to_string(dataSaver.getLaunchWriteAddress()));
//     cmdLine.print("Next Write Address: ");
//     cmdLine.println(std::to_string(dataSaver.getNextWriteAddress()));
//     cmdLine.print("Buffer Index: ");
//     cmdLine.println(std::to_string(dataSaver.getBufferIndex()));
//     cmdLine.print("Buffer Flushes: ");
//     cmdLine.println(std::to_string(dataSaver.getBufferFlushes()));

//     cmdLine.println("");
//     cmdLine.println("--Sensors--");
//     cmdLine.print("Accelerometer X: ");
//     cmdLine.println(floatToString(xAclData.getLastDataPointSaved().data));
//     cmdLine.print("Accelerometer Y: ");
//     cmdLine.println(floatToString(yAclData.getLastDataPointSaved().data));
//     cmdLine.print("Accelerometer Z: ");
//     cmdLine.println(floatToString(zAclData.getLastDataPointSaved().data));
//     cmdLine.print("Gyroscope X: ");
//     cmdLine.println(floatToString(xGyroData.getLastDataPointSaved().data));
//     cmdLine.print("Gyroscope Y: ");
//     cmdLine.println(floatToString(yGyroData.getLastDataPointSaved().data));
//     cmdLine.print("Gyroscope Z: ");
//     cmdLine.println(floatToString(zGyroData.getLastDataPointSaved().data));
//     cmdLine.print("Temperature: ");
//     cmdLine.println(floatToString(tempData.getLastDataPointSaved().data));
//     cmdLine.print("Pressure: ");
//     cmdLine.println(floatToString(pressureData.getLastDataPointSaved().data));
//     cmdLine.print("Altitude: ");
//     cmdLine.println(floatToString(altitudeData.getLastDataPointSaved().data));
//     cmdLine.print("Magnetometer X: ");
//     cmdLine.println(floatToString(xMagData.getLastDataPointSaved().data));
//     cmdLine.print("Magnetometer Y: ");
//     cmdLine.println(floatToString(yMagData.getLastDataPointSaved().data));
//     cmdLine.print("Magnetometer Z: ");
//     cmdLine.println(floatToString(zMagData.getLastDataPointSaved().data));
// }