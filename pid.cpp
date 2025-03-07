#include "pid.hpp"

float PID_dt = 0.020; // s
float Kp = 0.0;
float Ki = 0.0;
float Kd = 0.0;

float Ki_acc = 0.0;
float Kd_prev = -1.0; // init as invalid TOF reading to discern first iter

float pid_history[] = {0};

float clamp(float val, float low, float high)
{
  //
  if (val > high)
  {
    return high;
  }
  else if (val < low)
  {
    return low;
  }
  else
  {
    return val;
  }
}

float calculatePID(float pos, float set)
/*
 * Returns
 */
{
  float error = set - pos;
  Ki_acc += error * PID_dt;

  float derivative;
  if (Kd_prev != -1.000000000)
  {
    // NOT the first iteration -- proceed as normal
    derivative = (error - Kd_prev) / PID_dt;
  }
  else
  {
    // the first iteration -- skip Kd term.
    derivative = 0;
  }

  return clamp(Kp * error + Ki * Ki_acc + Kd * derivative, -255., 255.);
}