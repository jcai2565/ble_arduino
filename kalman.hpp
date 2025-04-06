#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <BasicLinearAlgebra.h>
using namespace BLA;

class KalmanFilter {
public:
    // Constructor
    KalmanFilter(float dt, float mass, float dist, float sigma_meas, float sigma_proc_1, float sigma_proc_2);

    // Prediction step (called every control cycle)
    void predict(float controlInput);

    // Update step (called only when new measurement is available)
    void update(float measurement);

    // Getters
    float getPosition() const;
    float getVelocity() const;

    private:
    float dt_;
    Matrix<2,2> A, Ad, sigma, sig_u, I;
    Matrix<2,1> B, Bd, x;
    Matrix<1,2> C;
    Matrix<1,1> sig_z;
};

#endif
