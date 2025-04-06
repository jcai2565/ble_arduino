#include "motors.hpp"
#include "utils.hpp"

// Motor strength percentage (Must be <1)
float left_percent = 1.0;
float right_percent = 0.95;

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

  if (pwm > 0)
  {
    drive(FORWARD, (int)clamp(pwm, DEADBAND, MAX_PWM));
  }
  else // pwm < 0, so need to negate
  {
    drive(BACKWARD, ((int)clamp(-pwm, DEADBAND, MAX_PWM)));
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