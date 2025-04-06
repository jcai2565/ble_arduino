#ifndef MOTORS_HPP
#define MOTORS_HPP

#include "pins.h"
#include "config.hpp"

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

// Motor Strength Adjustment factors
extern float left_percent;
extern float right_percent;

void motorSetup();
void motorOpenLoop();
void spin(Side side, int pwm);
void drive(Direction dir, int pwm);

void brakeFor(int ms);

void stop();
void cyclePwmTest();


/*
 * Converts the output of compute() and calls the drive() function to go forward or backwards.
 * REQUIRES that [pwm] is the output of pos_pid.compute().
 */
void executePosPid(int pwm);

/*
 * Converts the output of compute() and calls the spin() function to spin LEFT (ccw) or RIGHT (cw).
 * REQUIRES that [pwm] is the output of angle_pid.compute().
 */
void executeAnglePid(int pwm);

#endif