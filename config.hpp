#ifndef CONFIG_HPP
#define CONFIG_HPP

// Return array size
#define array_size 500
#define pid_array_size 2000

// Low pass filter
#define DO_LOWPASS true
#define imu_lowpass_alpha 0.284

// Complementary filter
#define alpha_complementary 0.1

// Motor pwm variables
#define DEADBAND 45
#define MAX_PWM 255.0
#define MIN_PWM -255.0

#define max_integral 300.

#define DO_DEBUG false

extern int timestamp_array[array_size];

#endif // CONFIG_HPP
