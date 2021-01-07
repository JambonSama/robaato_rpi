#pragma once

// C++ headers
#include <thread>

// ROS headers
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseAction.h>

// project headers
#include "panoramic_camera_worker.h"
#include "serial_port_worker.h"

enum class Robot { START, STARTED, RUNNING, STOP, n };
enum class Nav { OUTBOUND, INBOUND, OPEN, n };
enum class Roller { FORWARD, BACKWARD, STOP, n };

class RobotWorker {
private:
	// BASE CLASS MEMBERS
	Robot robot_;
	Nav navigation_;
	Roller roller_;

	CameraWorker *rpi_cam_;
	SerialPortWorker *serial_port_;

	ros::NodeHandle node_handle_;
	ros::Publisher nav_goal_publisher_;
	ros::Publisher goal_id_publisher_;

	// THREAD CLASS MEMBERS
	std::thread th_operate_;
	std::thread th_navigate_;
	std::thread th_capture_;
	std::thread th_publish_topics_;
	std::thread th_subscribe_to_topics_;

	// CLASS METHODS
	void Operate();
	void Navigate();
	void Capture();

	void PrintRobot();
	void PrintNavig();
	void PrintRolle();

	void Home();

	void PublishTopics();
	void SubscribeToTopics();
	bool moveToGoal(double xGoal, double yGoal, double yawGoal);

public:
	RobotWorker();
	virtual ~RobotWorker();
	void Start();
};

void NavStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &goal_status);

move_base_msgs::MoveBaseActionGoal Pose2Goal(Pose pose, std::string id);