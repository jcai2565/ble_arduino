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

/*
 * Cycles PWM to a pin, for testing motor drivers in lab 4.
 */
void cyclePwmTest();

/*
 * Executes the stunt cycle for lab 8.
 */
void stuntOpenLoop();

/*
 * Executes mapping sequence: changes robot setpoint in 10 degree increments and collect a few data points at each spot.
 * Stores all data in timestamp_array, distance1_array, and angle_pid's meas_array.
 * angle_pid.do_pid is NOT set ahead of time (nor is it relevant, because the command should execute a sequence and terminate).
 * REQUIRES: the PID Gains are set ahead of time, in a bluetooth function call.
 */
void mappingSequence(float incr, float error, int num_readings);

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

/**
 * Used to test the speed of motor.
 */
void driveForwardFor1Second();

/**
 * Sets the motor speed to adjust timings in Lab 12. Used in conjuction with driveForwardFor1Second()
 */
void setMotorSpeed(float spd);

/**
 * Used in Lab 12 to drive OL forward.
 * Input: [dist], in feet.
 * Drives forward for dist if [shouldCheckTof] is false.
 * Else, checks TOF reading every ~125 ms and returns early if TOF reading < 1 foot.
 */
void driveForwardDistance(float dist, bool shouldCheckTof);

/**
 * Uses PID to turn by [angle].
 * Counterclockwise is positive. Requires that PID gains for angle are set.
 * Input: [angle], in degrees, to turn by.
 */
void rotateByAngleWithPid(float angle);

void executeWaypointSequence();

extern float motorSpeed;

#endif