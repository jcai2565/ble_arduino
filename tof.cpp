#include "tof.hpp"
#include "pins.h"

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, XSHUT);

float distance1_array[array_size];
float distance2_array[array_size];

float getTof1IfReady()
{
  int d1;
  if (distanceSensor1.checkForDataReady())
  {
    d1 = distanceSensor1.getDistance();
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    distanceSensor1.startRanging();
  }
  else
  {
    d1 = -1.0;
  }
  return (float)d1;
}

float getTof2IfReady()
{
  int d2;
  if (distanceSensor2.checkForDataReady())
  {
    d2 = distanceSensor2.getDistance();
    distanceSensor2.clearInterrupt();
    distanceSensor2.stopRanging();
    distanceSensor2.startRanging();
  }
  else
  {
    d2 = -1.0;
  }
  return (float)d2;
}

float getTof1WithDelay()
{
  float d1 = getTof1IfReady();
  while (d1 == -1.0)
  {
    delay(1);
    d1 = getTof1IfReady();
  }
  return d1;
}

float getTof2WithDelay()
{
  float d2 = getTof2IfReady();
  while (d2 == -1.0)
  {
    delay(1);
    d2 = getTof2IfReady();
  }
  return d2;
}

void tofSetup()
{
  // TOF setup
  pinMode(XSHUT, OUTPUT); // write, to control XSHUT
  distanceSensor1.init();
  distanceSensor2.init();
  distanceSensor1.stopRanging();
  distanceSensor2.stopRanging();

  // Configure TOF1, disable TOF2
  digitalWrite(XSHUT, LOW);
  distanceSensor1.setI2CAddress(ALT_I2C); // set TOF 1 (the one without XSHUT) to ALT_I2C
  // Serial.println(distanceSensor1.getI2CAddress(), HEX);
  // Serial.println(distanceSensor2.getI2CAddress(), HEX);
  delay(10); // Allow TOF1 to boot
  // Reset XSHUT so that TOF2 is active
  digitalWrite(XSHUT, HIGH);
  delay(10); // Allow TOF2 to boot

  // Verify addresses changed
  //  Serial.print("Distance Sensor 1 Address: 0x");
  //  Serial.println(distanceSensor1.getI2CAddress(), HEX);
  //  Serial.print("Distance Sensor 2 Address: 0x");
  //  Serial.println(distanceSensor2.getI2CAddress(), HEX);

  int status1 = distanceSensor1.begin();
  Serial.print("Sensor 1 Status Code");
  Serial.println(status1);
  if (status1 != 0)
  {
    Serial.println("Distance sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  int status2 = distanceSensor2.begin();
  Serial.print("Sensor 2 Status Code");
  Serial.println(status2);
  if (status2 != 0)
  {
    Serial.println("Distance sensor 2 failed to begin. Please check wiring. Trying again...");
    while (1)
      ;
  }

  distanceSensor1.setDistanceModeLong();
  distanceSensor1.startRanging();
  distanceSensor2.setDistanceModeLong();
  distanceSensor2.startRanging();

  Serial.println("Both TOF sensors initialized successfully!");
}
