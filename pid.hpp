#ifndef PID_HPP
#define PID_HPP

// Shared PID values
extern float PID_dt;
extern float Kp;
extern float Ki;
extern float Kd;
extern float Ki_acc;
extern float Kd_prev;

extern float pid_history[];

float calculatePID(float pos, float set);

#endif // PID_HPP
