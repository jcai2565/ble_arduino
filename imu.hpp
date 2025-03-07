#ifndef IMU_HPP
#define IMU_HPP

#include "config.hpp"  // Ensure `array_size` is defined
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

// Declare IMU arrays (extern means they are defined elsewhere)
extern int timestamp_array[array_size];
extern float temperature_array[array_size];
extern float acc_roll_array[array_size];
extern float acc_pitch_array[array_size];

extern float gyr_roll_array[array_size];
extern float gyr_pitch_array[array_size];
extern float gyr_yaw_array[array_size];

extern float acc_roll_array_lowpass[array_size];
extern float acc_pitch_array_lowpass[array_size];

extern float complementary_roll_array[array_size];
extern float complementary_pitch_array[array_size];

extern float lowpass_pitch[2];
extern float lowpass_roll[2];

extern float gyr_pitch, gyr_roll, gyr_yaw, dt;
extern float complementary_pitch, complementary_roll;
extern unsigned long gyr_last;

#ifdef USE_SPI
extern ICM_20948_SPI myICM;
#else
extern ICM_20948_I2C myICM;
#endif

#endif // IMU_HPP
