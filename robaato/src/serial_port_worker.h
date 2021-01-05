#pragma once

// C++ headers
#include <mutex>
#include <thread>

// ROS headers
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// project headers
#include "message.h"

class SerialPortWorker {
protected:
	// BASE CLASS MEMBERS
	bool stop_ = false;
	int serial_port_;

	SensorMessage sensor_message_;
	ControlMessage control_message_;
	ros::NodeHandle node_handle_;

	// THREAD CLASS MEMBERS
	std::thread th_read_write_serial_port_;
	std::thread th_update_velocity_command_;
	std::thread th_broadcast_tfs_;
	std::thread th_publish_topics_;

	// CLASS METHODS
	void BroadcastTfs();
	void ConfigureSerialPort();
	void DisplayControlCommand();
	void DisplaySensorReadings();
	void PublishTopics();
	void ReadSensorMessage();
	void ReadWriteSerialPort();
	void UpdateVelocityCommand();
	void WriteControlMessage();

public:
	SerialPortWorker();
	~SerialPortWorker();
	void Start();
};

void VelocityCmdCallback(const geometry_msgs::Twist::ConstPtr &velocity_command);
