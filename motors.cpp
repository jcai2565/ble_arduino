#include "motors.hpp"

// Motor strength percentage (Must be <1)
float left_percent = 1;
float right_percent = 0.9;

void motorSetup()
{
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
}

void spin(Side side, int pwm)
{
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

void stop()
{
  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);
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

void driveFromPID(float pid_out)
{

}