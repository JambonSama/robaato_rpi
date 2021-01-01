#pragma once

// general c++ headers
#include <cstdint>
#include <mutex>
#include <thread>

// ros headers
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

// project headers
#include "message.h"

class SerialPortWorker {
protected:
    // BASE CLASS MEMBERS
    bool stop_ = false;
    SensorMessage sensor_message_;
    ControlMessage control_message_;
    ros::NodeHandle node_handle_;

    // THREAD CLASS MEMBERS
    std::thread th_read_write_serial_port_;
    std::thread th_get_command_;

    // CLASS METHODS
    void ReadWriteSerialPort();
    void GetCommand();

public:
    SerialPortWorker();
    ~SerialPortWorker();
    void Start();
};

void CmdCallback(const geometry_msgs::Twist::ConstPtr& velocity_command);
