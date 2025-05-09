#include "motors.hpp"
#include "utils.hpp"
#include "kalman.hpp"
#include "tof.hpp"
#include "config.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include <ArduinoBLE.h>

// Motor strength percentage (Must be <1)
float left_percent = 1.0;
float right_percent = 1.0;

// Trust that pos_kf is declared elsewhere, in .ino file
extern KalmanFilter pos_kf;

// Trust angle_pid is defined elsewhere in .ino file
extern PIDController angle_pid;

// Trust stunt things are cleared and declared elsewhere
extern int stunt_tof_index;
extern int stunt_kf_index;
extern unsigned long stunt_start_time;

// Trust mapping things dec.
extern int mapping_index;

void motorSetup()
{
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
}

void spin(Side side, int pwm)
{
  // ccw
  if (side == LEFT)
  {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, pwm);
    analogWrite(M2_IN1, pwm);
    analogWrite(M2_IN2, 0);
  }
  else if (side == RIGHT)
  {
    analogWrite(M1_IN1, pwm);
    analogWrite(M1_IN2, 0);
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, pwm);
  }
}

void drive(Direction dir, int pwm)
{
  if (dir == FORWARD)
  {
    analogWrite(M1_IN1, int(pwm * left_percent));
    analogWrite(M1_IN2, 0);
    analogWrite(M2_IN1, int(pwm * right_percent));
    analogWrite(M2_IN2, 0);
  }
  else if (dir == BACKWARD)
  {
    analogWrite(M1_IN1, 0);
    analogWrite(M1_IN2, int(pwm * left_percent));
    analogWrite(M2_IN1, 0);
    analogWrite(M2_IN2, int(pwm * right_percent));
  }
}

void brake()
{
  analogWrite(M1_IN1, 255);
  analogWrite(M1_IN2, 255);
  analogWrite(M2_IN1, 255);
  analogWrite(M2_IN2, 255);
}

void stop()
{
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
}

void brakeFor(int ms)
{
  brake();
  delay(ms);
  stop();
}

void cyclePwmTest()
{
  for (int i = 0; i < 255; i++)
  {
    analogWrite(M1_IN1, i);
    analogWrite(M2_IN1, i);
  }
  analogWrite(M1_IN1, 0);
  analogWrite(M2_IN1, 0);

  delay(500);

  for (int i = 0; i < 255; i++)
  {
    analogWrite(M1_IN2, i);
    analogWrite(M2_IN2, i);
  }
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN2, 0);
}

void motorOpenLoop()
{
  drive(FORWARD, 235);
  delay(750);
  stop();
  delay(2500);

  spin(RIGHT, 200);
  delay(1000);
  stop();
  delay(2500);

  drive(FORWARD, 200);
  delay(750);
  stop();
  delay(2500);

  spin(LEFT, 200);
  delay(1000);
  stop();
  delay(2500);

  delay(5000);
}

/*
 * Performs the stunt, and is called once during the bluetooth command.
 * KF data is stored within pos_kf's distance array. Index using stunt_kf_index;
 *  ... corresponding time is stored within global timestamp_array.
 * TOF data is stored within global distance1_array. Index using stunt_tof_index;
 *  ... corresponding time is stored within global big_timestamp_array.
 */
void stuntOpenLoop()
{
  const float flipDistance = 1200; // mm
  const float maxRunTime = 10000;  // ms

  // 1. Start by driving forward against the wall by default.
  drive(FORWARD, 255);

  // 2. Initialize and run Kalman filter. Stay in the loop until we reach threshold
  float d1 = getTof1IfReady();
  while (1)
  {
    unsigned long cur_time = millis();

    if (d1 != -1.0)
    {
      timestamp_array[stunt_tof_index] = cur_time;
      distance1_array[stunt_tof_index] = d1;
      stunt_tof_index++;

      pos_kf.update(d1);
    }
    // Note this stores to pos_kf's array for distance.
    // Also note negation, see lab 7.
    pos_kf.predict(KalmanFilter::normalize(-255));

    float cur_pos = pos_kf.getPosition();
    big_timestamp_array[stunt_kf_index] = cur_time;
    pos_kf.position_array[stunt_kf_index] = cur_pos;
    stunt_kf_index++;

    if (cur_pos < flipDistance || cur_time - stunt_start_time > maxRunTime)
    {
      break; // out of the while loop
    }

    d1 = getTof1IfReady();
  }

  // 3. Run the robot in reverse and drive for an open loop time (after this point data is irrelevant).
  const float reverseTiming = 3000; // ms
  drive(BACKWARD, 255);
  delay(reverseTiming);
  stop();
  return;
}

void executePosPid(int pwm)
{
  // if (abs(pwm) < 3)
  // {
  //   stop();
  return;
  // }

  int input;
  if (pwm > 0)
  {
    // cast pwm to float for more accurate fp division?
    input = (int)clamp((float)pwm, DEADBAND, MAX_PWM);

    // Predict step every control
    // NOTE: a positive u suggests that distance is increasing, so we should negate u if it leads to decreasing dist.
    pos_kf.predict(KalmanFilter::normalize(-input)); // Feed control signal to KF

    drive(FORWARD, input);
  }
  else // pwm < 0, so need to negate
  {
    // This should be a positive
    input = (int)clamp(-pwm, DEADBAND, MAX_PWM);

    // Need to negate back to negative
    // NOTE: a positive u suggests that distance is increasing, so we should negate u if it leads to decreasing dist and vice versa
    pos_kf.predict(KalmanFilter::normalize(input)); // Feed control signal to KF

    // positive for motor input
    drive(BACKWARD, input);
  }
}

void executeAnglePid(int pwm)
{
  if (abs(pwm) < 3)
  {
    stop();
    return;
  }
  // If angle > setpoint, then PWM passed in will be >0, meaning we should rotate CW to decrease our angle with +ve CCW convention
  // Use a higher deadband for angle because on regular deadband one side turns but the other does not.
  if (pwm > 0)
  {
    spin(RIGHT, (int)clamp(pwm + DEADBAND, 0., MAX_PWM));
  }
  else // pwm < 0, so need to negate
  {
    spin(LEFT, ((int)clamp(-pwm + DEADBAND, 0., MAX_PWM)));
  }
}

void mappingSequence(float incr, float error, int num_readings)
{
  // Note that BLE.poll() is added to keep BLE connection alive during long sequence
  //
  // Increment setpoint from 0 to 360 by [incr]
  for (float sp = 0.0; sp < 361.; sp += incr)
  {
    if (DO_DEBUG)
    {
      Serial.print("Setpoint set:");
      Serial.println(sp);
    }

    angle_pid.setSetpoint(sp);

    // Get to within +-[error] of the setpoint
    float angle = getValidDmpYaw();

    if (DO_DEBUG)
    {
      Serial.print("Current angle is: ");
      Serial.print(angle);
      Serial.print(" | ");
      Serial.print("Current error is: ");
      Serial.println(abs(angle - sp));
    }

    while (abs(angle - sp) >= error)
    {
      angle = getValidDmpYaw();
      int pwm = angle_pid.compute(angle);
      executeAnglePid(pwm);
      BLE.poll();
    }

    // Once we've reached error bounds
    stop();
    angle_pid.resetAccumulator(); // prevent integral windup

    // Take readings from TOF, storing pairwise datapoints.
    int i = 0;
    while (i < num_readings)
    {
      float d1 = getTof1IfReady();
      if (d1 == -1.0)
      {
        BLE.poll();
        continue; // Spins until TOF ready
      }

      // Since DMP is faster we can reasonably assume no massive delays once TOF ready
      float angle = getValidDmpYaw();

      // Store angle and distance once we have valid d1 and angle
      timestamp_array[mapping_index] = millis();
      angle_pid.meas_array[mapping_index] = angle;
      distance1_array[mapping_index] = d1;
      mapping_index++;
      BLE.poll();

      i++;
    }
  }

  stop();
  return;
}