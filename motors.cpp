#include "motors.hpp"
#include "utils.hpp"
#include "kalman.hpp"
#include "tof.hpp"
#include "config.hpp"

// Motor strength percentage (Must be <1)
float left_percent = 1.0;
float right_percent = 1.0;

// Trust that pos_kf is declared elsewhere, in .ino file
extern KalmanFilter pos_kf;

// Trust stunt things are cleared and declared elsewhere
extern int stunt_tof_index;
extern int stunt_kf_index;
extern unsigned long stunt_start_time;

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

  const float reverseTiming = 3000; // ms

  // pre3. active brake to flip
  // brakeFor(500); // ms
  // 3. Run the robot in reverse and drive for an open loop time (after this point data is irrelevant).
  drive(BACKWARD, 255);
  delay(reverseTiming);
  stop();
  return;
}

void executePosPid(int pwm)
{

  // Because we never get to an absolute zero.
  if (abs(pwm) < 3)
  {
    stop();
  }

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
  if (pwm > 0)
  {
    pwm += DEADBAND;
    spin(RIGHT, (int)clamp(pwm, MIN_PWM, MAX_PWM));
  }
  else // pwm < 0, so need to negate
  {
    pwm -= DEADBAND;
    spin(LEFT, -((int)clamp(pwm, MIN_PWM, MAX_PWM)));
  }
}