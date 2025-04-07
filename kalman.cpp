#include "kalman.hpp"

KalmanFilter::KalmanFilter(float dt, float mass, float dist, float sigma_meas, float sigma_proc_1, float sigma_proc_2)
    : dt_(dt)
{
  // Continuous system
  A = {0, 1,
       0, -dist / mass};

  B = {0,
       1 / mass};

  // Identity matrix
  I = {1, 0,
       0, 1};

  // Measurement matrix
  C = {1, 0};

  // Discretized dynamics
  Ad = I + A * dt_;
  Bd = B * dt_;

  // Initial state
  x = {0, 0};

  // Initial covariance
  sigma = {400, 0,
           0, 1};

  // Process noise
  float q1 = sigma_proc_1 * sigma_proc_1;
  float q2 = sigma_proc_2 * sigma_proc_2;
  sig_u = {q1, 0,
           0, q2};

  // Measurement noise
  float r = sigma_meas * sigma_meas;
  sig_z = {r};

  // Clean measurement array
  for (int i = 0; i < pid_array_size; i++)
  {
    position_array[i] = 0.0f;
  }
}

void KalmanFilter::predict(float controlInput)
{
  Matrix<1, 1> u = {controlInput};
  x = Ad * x + Bd * u;
  sigma = Ad * sigma * ~Ad + sig_u;

  if (kf_index < pid_array_size)
  {
    position_array[kf_index] = x(0, 0); // log filtered position
    Serial.print("KF Pos:");
    Serial.print(x(0, 0));
    Serial.print(" | ");
    Serial.print("KF Velocity:");
    Serial.println(x(1,0));
    kf_index++;
  }
  else
  {
    Serial.println("KF array is full!");
  }
}

void KalmanFilter::update(float measurement)
{
  Matrix<1, 1> z = {measurement};
  Matrix<1, 1> S = C * sigma * ~C + sig_z;

  // Scalar inverse because S is 1x1
  Matrix<2, 1> K = sigma * ~C * (1.0f / S(0, 0));
  Matrix<1, 1> y = z - C * x;

  x = x + K * y;
  sigma = (I - K * C) * sigma;
}

void KalmanFilter::initialize(float firstMeasurement)
{
  x = {firstMeasurement, 0};
  //I trust position to be initially accurate to TOF's 5mm +-
  sigma = {25, 0, 0, 1};
  kf_index = 0;
  for (int i = 0; i < pid_array_size; i++)
  {
    position_array[i] = 0.0f;
  }
}

float KalmanFilter::normalize(int pwm)
{
  return (float)pwm / 150.;
}

float KalmanFilter::getPosition() const
{
  return x(0, 0);
}

float KalmanFilter::getVelocity() const
{
  return x(1, 0);
}
