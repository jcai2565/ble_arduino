#include "imu.hpp"
#include <math.h> //for atan2, sin in Quat6 -> E.Angle

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

//
// 2 Point Calibration
//
float calibratedRoll(float val)
{
  const float measuredMinus90 = -87.5;
  const float measuredPlus90 = 88;

  return -90 + (val - measuredMinus90) * (180 / (measuredPlus90 - measuredMinus90));
}

float calibratedPitch(float val)
{
  const float measuredMinus90 = -85;
  const float measuredPlus90 = 87;

  return -90 + (val - measuredMinus90) * (180 / (measuredPlus90 - measuredMinus90));
}

// SETUP

/*
 * Helper to set up DMP, called in imuSetup()
 */
void dmpSetup()
{
  // From Example7_DMP_Quat6_EulerAngles, taken from Stephan's tutorial.
  // https://github.com/FastRobotsCornell/FastRobots-2025/blob/main/docs/tutorials/dmp.md
  bool success = true;

  // Initialize the DMP
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
  // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  // arg 0 means that we desire 55 Hz data output rate:
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0.1) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO queue
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (!success)
  {
    Serial.println("Enabling DMP failed!");
    while (1)
    {
      // Freeze
    }
  }
}

float lastValue = 0.;
bool lastValueInitialized = false;

/*
 * Reads Quat6 data from DMP and converts into Euler Angle.
 * Returns: [yaw], the yaw angle in degrees, in absolute orientation, the result of converting the Quat6 data from DMP.
 * Returns: 404404. , if the DMP FIFO has no data available. Use previous data point in that case or extrapolate.
 * Returns: 1000000. + sensor status, if the sensor status is not OK.
 * Returns: -666666.0, if myICM.status is unexpected (default return).
 */
float getDmpYaw()
{
  // From Example7_DMP_Quat6_EulerAngles, taken from Stephan's tutorial.
  // https://github.com/FastRobotsCornell/FastRobots-2025/blob/main/docs/tutorials/dmp.md

  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  // Keep reading until we get to the latest element in the FIFO queue, with a limit of [maxReads] times.
  int maxReads = 10;
  while (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail && maxReads-- > 0)
  {
    myICM.readDMPdataFromFIFO(&data);
  }

  if (myICM.status == ICM_20948_Stat_FIFONoDataAvail)
  {
    return 404404.;
  }

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
    // We have asked for GRV data so we should receive Quat6
    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - (q1 * q1 + q2 * q2 + q3 * q3));

      // Convert the quaternion to Euler angles. We only need [yaw], but all 3 here for completeness.
      // double roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
      // double pitch = asin(2.0 * (q0 * q2 - q3 * q1));
      double yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3));

      // Convert radians to degrees
      // roll *= 180.0 / M_PI;
      // pitch *= 180.0 / M_PI;
      yaw *= 180.0 / M_PI;

      // Changes output from [-180, 180] to any angle by assuming continuity (the last value cannot differ by more than 360. than the current value naturally).
      if (!lastValueInitialized)
      {
        lastValue = yaw;
        lastValueInitialized = true;
        return yaw;
      }
      else
      {
        // Assume that the 55Hz sensor isn't moving 180. degrees between measurements
        while (yaw - lastValue >= 180.)
        {
          yaw -= 360.;
        }
        while (yaw - lastValue <= -180.)
        {
          yaw += 360.;
        }
        lastValue = yaw;
      }

      return yaw;
    }
  }
  // myICM status is not ok, should also not reach because MoreDataAvail and NoDataAvail are both checked for..
  else if (myICM.status == ICM_20948_Stat_WrongID)
  {
    return 100000. + myICM.status; // bad
  }

  return -666666.; // default return value, should not reach.
}

bool isValidYaw(float yaw)
{
  return yaw > -99999. && yaw < 99999.;
}

float getValidDmpYaw()
{
  float angle = getDmpYaw();
  while (!isValidYaw(angle))
  {
    delay(1);
    angle = getDmpYaw();
  }
  return angle;
}

void imuSetup()
{
#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  // SENSOR INITIALIZATION CHECKS
  bool initialized = false;
  while (!initialized)
  {
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
  dmpSetup();
}
