#include "controller.h"

#include <iostream>

namespace {
// queue size for suscribed values
    const uint32_t queue_size = 50;
}

Controller::Controller(){}

Controller::~Controller(){
    stop_ = true;

    if(th_control_.joinable()){
        th_control_.join();
    }
}

void Controller::Control(){
    ros::Subscriber command_suscriber = node_handle_.subscribe("cmd_vel", queue_size, CmdCallback);
    while(!stop_){
        ros::spinOnce();
        std::this_thread::sleep_for(3ms);
    }
}

void Controller::Start(){
    th_control_ = std::thread(&Controller::Control, this);
}

void CmdCallback(const geometry_msgs::Twist::ConstPtr& velocity_command){
    std::cout << velocity_command->linear << std::endl;
    std::cout << velocity_command->angular << std::endl;
}
