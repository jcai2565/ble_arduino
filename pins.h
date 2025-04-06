#ifndef PINS_H
#define PINS_H

#define SERIAL_PORT Serial

// SPI ALTENATIVE
//#define USE_SPI       // Uncomment this to use SPI

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
//SPI ALTERNATIVE

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

/*
8 Lab 3: TOF
*/

//MY XSHUT
#define XSHUT 8 

// //SOPHIA's XSHUT
// #define XSHUT 2

#define ALT_I2C 0x30 //arbitrary within range 0x08 to 0x77; must be 7-bit valid

/*
* Lab 4: Motors
*/
//MY MOTORS
//Left
#define M1_IN1 5
#define M1_IN2 2

//Right
#define M2_IN1 12
#define M2_IN2 15

// // SOPHIA'S MOTORS
// #define M1_IN1 1
// #define M1_IN2 15

// //Right
// #define M2_IN1 16
// #define M2_IN2 5


// //ADRIENNE'S MOTORS
// //Left
// #define M1_IN1 0
// #define M1_IN2 1

// //Right
// #define M2_IN1 4
// #define M2_IN2 3

#endif //PINS_H