#ifndef TOF_HPP
#define TOF_HPP

#include <Wire.h>
#include "SparkFun_VL53L1X.h" // Include the SparkFun VL53L1X library
#include "config.hpp"
#include "pins.h"

// TOF sensor objects as extern so they can be defined elsewhere
extern SFEVL53L1X distanceSensor1;
extern SFEVL53L1X distanceSensor2;

// TOF arrays
extern float distance1_array[array_size];
extern float distance2_array[array_size];

/*
* Returns: a float, the distance (in mm) recorded by TOF1 if the data is ready,
* Returns: -1.0 if the data is not ready.
*/
float getTof1IfReady();
float getTof2IfReady();

void tofSetup();

#endif // TOF_HPP
