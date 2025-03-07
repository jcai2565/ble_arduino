#ifndef UTILS_HPP
#define UTILS_HPP

#include <Arduino.h>
#include "ICM_20948.h"
#include "pins.h"

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