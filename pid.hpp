#ifndef PID_HPP
#define PID_HPP

#include "config.hpp"
#include "lpf.hpp"
#include "motors.hpp"

class PIDController
{
public:
  float setpoint;
  float Kp, Ki, Kd;
  float pid_dt;
  float accumulator;
  float prev_val;

  // pid_index is used on array_size, for slower sensor readings
  int pid_index;
  int pwm_history[array_size];
  int time_array[array_size];   // Stores timestamps for measurements
  float meas_array[array_size]; // Measurement array that stores either angles or distances based on the PID type
  float setpoint_array[array_size];

  // pid_control_index is used on pid_array_size, for faster control loop.
  int pid_control_index;
  int pid_timestamp_array[pid_array_size];
  float p_array[pid_array_size];
  float i_array[pid_array_size];
  float d_array[pid_array_size];

  bool do_pid;
  unsigned long pid_start_time;
  unsigned long pid_prev_time;

  LowPassFilter derivativeFilter; // LPF for derivative term

  // Constructor
  PIDController(float sp = 0.0, float kp = 0.0, float ki = 0.0, float kd = 0.0, float dt = 0.02, float alpha = 0.1)
      : setpoint(sp), Kp(kp), Ki(ki), Kd(kd), pid_dt(dt), accumulator(0.0), prev_val(-1.0),
        pid_index(0), pid_control_index(0), do_pid(false), pid_start_time(0), pid_prev_time(0), derivativeFilter(alpha) // Initialize LPF with given alpha
  {
    // Initialize arrays to zero
    for (int i = 0; i < array_size; i++)
      pwm_history[i] = 0;
    for (int i = 0; i < pid_array_size; i++)
    {
      pid_timestamp_array[i] = 0;
      p_array[i] = 0.0;
      i_array[i] = 0.0;
      d_array[i] = 0.0;
    }
  }

  /*
   * Sets PID gains dynamically.
   */
  void setGains(float kp, float ki, float kd);

  /*
   * Sets a new setpoint dynamically.
   */
  void setSetpoint(float sp);

  /*
   * Returns a PWM value in [-255, 255]. Negative indicates motor should drive backwards.
   * ONLY CALL THIS ONCE PER LOOP!
   */
  int compute(float pos);

  /*
   * Calculates the linear extrapolation from (timestamp_array[pid_index-2], meas_array[pid_index-2]) (timestamp_array[pid_index-1], meas_array[pid_index-1]).
   * To predict what the next read value should be.
   * Returns: the predicted next value.
   */
  float linearExtrapolate();

  /*
   * Resets all PID-related parameters and prepares for a fresh PID cycle.
   * The flag PIDController::do_pid should still be set by the BLE command.
   */
  void reset();
};

// Declare two external PID controllers
extern PIDController angle_pid;
extern PIDController pos_pid;

#endif // PID_HPP
