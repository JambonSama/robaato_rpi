#pragma once

#include <cstdint>

#define TOF_SENSOR_NUM (6)
#define IMU_QUANTITIES_NUM (9)

struct __attribute__((__packed__)) ControlMessage {
    bool gate_position = false;        //(=0 for CLOSED) (=1 for OPEN)
    bool left_wheel_direction = true;  //(=1:forward) (=0: backward)
    bool right_wheel_direction = true; //(=1:forward) (=0: backward)
    float left_wheel_speed = 0;        //[rpm] range:0-3000
    float right_wheel_speed = 0;       //[rpm] range:0-3000
    uint8_t roller_state = 0;          //(=0: OFF) (=1: FORWARD) (=2: BACKWARD)
};

struct __attribute__((__packed__)) SensorMessage {
	float imu[IMU_QUANTITIES_NUM] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // a_x, a_y, a_z, roll, pitch, yaw, v_roll, v_pitch, v_yaw
    float tof_sensors[TOF_SENSOR_NUM] = {0, 0, 0, 0, 0, 0};      // FM FR FL SR SL BM
    bool bottle_sensor = 0;                                      //(=0: No bottle detected) (=1: bottle detected)
    bool roller_encoder_state = 0;                               //(=0: Roller is off) (=1: roller is on)
};
