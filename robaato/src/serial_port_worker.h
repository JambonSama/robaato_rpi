#pragma once

// C++ headers
#include <cstdint>
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
	uint32_t bottle_number_ = 0;

	SensorMessage sensor_message_;
	ControlMessage control_message_;

	ros::NodeHandle node_handle_;

	// THREAD CLASS MEMBERS
	std::thread th_read_write_serial_port_;
	std::thread th_broadcast_tfs_;
	std::thread th_publish_topics_;
	std::thread th_subscribe_to_topics_;

	// CLASS METHODS
	void ConfigureSerialPort();

	void DisplaySensorReadings();
	void DisplayControlCommand();

	void ReadSensorMessage();
	void WriteControlMessage();
	void ReadWriteSerialPort();

	void BroadcastTfs();
	void PublishTopics();
	void SubscribeToTopics();

public:
	SerialPortWorker();
	~SerialPortWorker();
	void Start();

	// GETTERS
	uint32_t GetBottleNumber();
	bool GetRollerEncoderState();

	// SETTERS
	void SetNullControlMessage();
	void SetBottleNumber(uint32_t bottle_number);
	void SetGatePosition(bool gate_position);
	void SetRollerState(uint8_t roller_state);
};

void VelocityCmdCallback(const geometry_msgs::Twist::ConstPtr &velocity_command);
