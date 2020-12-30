#pragma once

// general c++ headers
#include <thread>
#include <thread>

// ros headers
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std::chrono_literals;

class Controller {
    protected:
    // BASE CLASS MEMBERS
    bool stop_ = false;
    ros::NodeHandle node_handle_;

    // THREAD CLASS MEMBERS
    std::thread th_control_;

    // MUTEX CLASS MEMBERS

    // CLASS METHODS

    public:
    Controller();
    virtual ~Controller();
    void Control();
    void Start();
};

void CmdCallback(const geometry_msgs::Twist::ConstPtr& velocity_command);
