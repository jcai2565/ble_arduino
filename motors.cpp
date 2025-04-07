#include "motors.hpp"
#include "utils.hpp"
#include "kalman.hpp"

// Motor strength percentage (Must be <1)
float left_percent = 1.0;
float right_percent = 1.0;

// Trust that pos_kf is declared elsewhere, in .ino file
extern KalmanFilter pos_kf;

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

void breakFor(int ms)
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