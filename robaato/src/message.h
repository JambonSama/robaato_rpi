#pragma once

#include <cstdint>

#define TOF_SENSOR_NUM (6)

struct __attribute__((__packed__)) ControlMessage {
    bool gate_position = false;        //(=0 for CLOSED) (=1 for OPEN)
    bool left_wheel_direction = true;  //(=1:forward) (=0: backward)
    bool right_wheel_direction = true; //(=1:forward) (=0: backward)
    float left_wheel_speed = 0;        //[rpm] range:0-3000
    float right_wheel_speed = 0;       //[rpm] range:0-3000
    uint8_t roller_state = 0;          //(=0: OFF) (=1: FORWARD) (=2: BACKWARD)
};

struct __attribute__((__packed__)) SensorMessage {
    float ax = 0;
    float ay = 0;
    float az = 0;
    float yaw = 0;
    float pitch = 0;
    float roll = 0;
    float tof_sensors[TOF_SENSOR_NUM] = {0, 0, 0, 0, 0, 0}; // FM FR FL SR SL BM
    bool bottle_sensor = 0; //(=0: No bottle detected) (=1: bottle detected)
    bool roller_encoder_state = 0;          //(=0: Roller is off) (=1: roller is on)
};
