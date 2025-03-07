#ifndef MOTORS_HPP
#define MOTORS_HPP

#include "pins.h"

enum Side
{
  LEFT,
  RIGHT,
};

enum Direction
{
  FORWARD,
  BACKWARD,
};

// Motor Strength Adjustment
extern float left_percent;
extern float right_percent;

void motorSetup();
void motorOpenLoop();
void spin(Side side, int pwm);
void drive(Direction dir, int pwm);
void stop();
void cyclePwmTest();
void driveFromPID(float pid_out);

#endif