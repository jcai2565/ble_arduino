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
}

void KalmanFilter::predict(float controlInput)
{
  Matrix<1, 1> u = {controlInput};
  x = Ad * x + Bd * u;
  sigma = Ad * sigma * ~Ad + sig_u;
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

float KalmanFilter::getPosition() const
{
  return x(0, 0);
}

float KalmanFilter::getVelocity() const
{
  return x(1, 0);
}
