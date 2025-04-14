#include "pid.hpp"
#include "utils.hpp"
#include "lpf.hpp"

// Initialize two PID controllers
// (float sp = 0.0, float kp = 0.0, float ki = 0.0, float kd = 0.0, float dt = 0.02, float alpha = 0.1)
PIDController angle_pid(0.0, 0.0, 0.0, 0.0, 0.02, 0.01); // Default values, to be adjusted
PIDController pos_pid(304.0, 0.0, 0.0, 0.0, 0.02, 0.05); // Setpoint of 304mm

int PIDController::compute(float pos)
{
  float error = pos - setpoint;
  unsigned long cur_time = millis();

  if (pid_prev_time != 0)
  {
    pid_dt = (cur_time - pid_prev_time) / 1000.0; // sec
  }
  pid_prev_time = cur_time;

  accumulator += error * pid_dt;

  float raw_derivative;

  if (prev_val != -1.000000000)
  {
    // NOT the first iteration -- proceed as normal
    // Anti-derivative kick: replace dError/dt with -dInput/dt
    // Question: is this necessary after LPF?

    // If pos < prev_val, we are speeding towards the wall, which should contribute a minus PWM.
    raw_derivative = (pos - prev_val) / pid_dt;
  }
  else
  {
    // the first iteration -- skip Kd term.
    raw_derivative = 0;
  }

  // Apply Low-Pass Filter to the derivative term
  float filtered_derivative = derivativeFilter.update(raw_derivative);
  prev_val = pos; // Save input for -dInput/dt of next loop

  float p = Kp * error;
  float i = Ki * accumulator;
  float d = Kd * filtered_derivative; // Use filtered derivative

  pid_timestamp_array[pid_control_index] = cur_time;
  p_array[pid_control_index] = p;
  i_array[pid_control_index] = i;
  d_array[pid_control_index] = d;
  Serial.print("PID Control--- Pos: ");
  Serial.print(pos);
  Serial.print(" | Error: ");
  Serial.print(error);
  Serial.print(" | P: ");
  Serial.print(p);
  Serial.print(" | ");
  Serial.print("I: ");
  Serial.print(i);
  Serial.print(" | ");
  Serial.print("D: ");
  Serial.print(d);
  Serial.print(" | ");
  Serial.print("dt:");
  Serial.print(pid_dt);
  Serial.print(" | ");
  Serial.print("accum:");
  Serial.print(accumulator);
  Serial.print(" | ");
  Serial.print("Total output: ");
  Serial.print((int)clamp(p + i + d, MIN_PWM, MAX_PWM));
  Serial.println("");

  pid_control_index++;

  return (int)clamp(p + i + d, MIN_PWM, MAX_PWM);
}

// Set PID gains dynamically
void PIDController::setGains(float kp, float ki, float kd)
{
  Kp = kp;
  Ki = ki;
  Kd = kd;
}

// Set a new setpoint dynamically
void PIDController::setSetpoint(float sp)
{
  setpoint = sp;
}

// Linear extrapolation based on meas_array and time_array
float PIDController::linearExtrapolate()
{
  // No extrapolation if one data point -- just use that data point
  if (pid_index <= 1)
  {
    return meas_array[pid_index];
  }
  else if (pid_index >= array_size) // Prevent out-of-bounds access
  {
    Serial.println("ARRAY OUT OF BOUNDS REACHED IN LINEAR EXTRAPOLATE");
    return meas_array[array_size - 1];
  }
  else
  {
    float slope = (meas_array[pid_index - 1] - meas_array[pid_index - 2]) /
                  (time_array[pid_index - 1] - time_array[pid_index - 2]); // mm or deg / ms
    float local_dt = millis() - time_array[pid_index - 1];                 // ms
    // Serial.print("Result of linear extrapolation: ");
    // Serial.println(meas_array[pid_index - 1] + slope * local_dt);
    return meas_array[pid_index - 1] + slope * local_dt;
  }
}

void PIDController::reset()
{
  pid_dt = 0.02; // s, default value
  accumulator = 0.0;
  prev_val = -1.000; // Init as invalid TOF reading to discern first iter

  pid_index = 0; // Sets return array index to 0
  pid_control_index = 0;

  for (int i = 0; i < array_size; i++)
  {
    time_array[i] = 0; // Replacing old timestamp array
    pwm_history[i] = 0;
    meas_array[i] = 0.0; // Replacing old distance1_array
    setpoint_array[i] = 0.0;
  }

  for (int i = 0; i < pid_array_size; i++)
  {
    pid_timestamp_array[i] = 0;
    p_array[i] = 0.0;
    i_array[i] = 0.0;
    d_array[i] = 0.0;
  }

  pid_start_time = millis();
}

// Serial.print("position: ");
// Serial.print(pos);
// Serial.print(" | ");
// Serial.print("p term:");
// Serial.print(p);
// Serial.print("| ");
// Serial.print("i term");
// Serial.print(i);
// Serial.print("| ");
// Serial.print("d term");
// Serial.println(d);