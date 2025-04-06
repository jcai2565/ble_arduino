#ifndef UTILS_HPP
#define UTILS_HPP

#include <Arduino.h>
#include "ICM_20948.h"
#include "pins.h"
#include "config.hpp"

// Returns true if [index] is between 0 and [array_size]
bool indexOutOfBounds(int index);

// Clamps a value between [low, high]
float clamp(float val, float low, float high);

// Function declarations
void printPaddedInt16b(int16_t val);
void printRawAGMT(ICM_20948_AGMT_t agmt);
void printFormattedFloat(float val, uint8_t leading, uint8_t decimals);

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor);
#else
void printScaledAGMT(ICM_20948_I2C *sensor);
#endif

#endif