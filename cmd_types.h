#ifndef CMD_TYPES_H
#define CMD_TYPES_H

enum CommandTypes {
    WRITE_IMU_DATA,
    SEND_IMU_DATA,
    GET_IMU_DATA,
    GET_TOF_DATA,
    GET_SENSOR_DATA,
    SET_KP,
    SET_KI,
    SET_KD,
};

#endif // CMD_TYPES_H