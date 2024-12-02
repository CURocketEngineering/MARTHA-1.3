#ifndef MARTHA_PINS_H
#define MARTHA_PINS_H

#include <Arduino.h>

#ifdef PCB_MARTHA

#define SENSOR_MISO PA6 
#define SENSOR_MOSI PA7 
#define SENSOR_SCK PA5
#define SENSOR_BARO_CS PB2
#define SENSOR_LSM_CS PB0 
#define SENSOR_LIS_CS PA2

#endif // PCB_MARTHA


#ifdef BB_MARTHA

#define SENSOR_MISO PB4 
#define SENSOR_MOSI PB5
#define SENSOR_SCK PB3
#define SENSOR_BARO_CS PA1
#define SENSOR_LSM_CS PA0
#define SENSOR_LIS_CS PA3

#endif // BB_MARTHA pins 
 

#endif 