#include "imu.hpp"

// Define IMU arrays (memory is allocated here)
int timestamp_array[array_size];
float temperature_array[array_size];
float acc_roll_array[array_size];
float acc_pitch_array[array_size];

float gyr_roll_array[array_size];
float gyr_pitch_array[array_size];
float gyr_yaw_array[array_size];

float acc_roll_array_lowpass[array_size];
float acc_pitch_array_lowpass[array_size];

float complementary_roll_array[array_size];
float complementary_pitch_array[array_size];

float lowpass_pitch[2];
float lowpass_roll[2];

float gyr_pitch = 0, gyr_roll = 0, gyr_yaw = 0, dt = 0;
float complementary_pitch = 0, complementary_roll = 0;
unsigned long gyr_last = 0;

#ifdef USE_SPI
ICM_20948_SPI myICM; 
#else
ICM_20948_I2C myICM;  
#endif