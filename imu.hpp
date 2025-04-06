#ifndef IMU_HPP
#define IMU_HPP

#include "config.hpp" // Ensure `array_size` is defined
#include "pins.h"
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

// Declare IMU arrays (extern means they are defined elsewhere)
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

void imuSetup();
float calibratedRoll(float val);
float calibratedPitch(float val);
float lowPassPitch(float val);
float lowPassRoll(float val);

/*
 * Reads Quat6 data from DMP and converts into Euler Angle.
 * Returns: [yaw], the yaw angle in degrees, in absolute orientation, the result of converting the Quat6 data from DMP.
 * Returns: 404. , if the DMP FIFO has no data available. Use previous data point in that case or extrapolate.
 * Returns: 1000. + sensor status, if the sensor status is not OK.
 * Returns: -666.0, if myICM.status is unexpected (default return).
 */
float getDmpYaw();

/*
 * Returns true if [yaw] is a valid yaw reading.
 */
bool isValidYaw(float yaw);

#endif

#endif // IMU_HPP
