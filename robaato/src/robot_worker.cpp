// C++ library headers
#include <cstring>
#include <iostream>

// ROS headers
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// project headers
#include "definitions.h"
#include "robot_worker.h"

namespace {
// timing
ros::Time initial_time;
ros::Time current_time;
const double running_time = 30;   // 30;   // 30 s for the arduino to be on
const double inbound_time = 90;   // 90;   // stops navigating after 8 min = 480 s
const double stop_time = 120;     // end of competition after 600 s = 10 min
const double roller_dt = 5;       // 5 s backward if stuck
const double door_dt = 10;        // 10 s to unload the bottles
const double refraction_time = 5; //

// robot states
const uint32_t max_bottle_number = 5; // max 5 bottles in the chassis
bool is_home = false;

// navigation
const uint32_t nav_goal_queue_size = 20;
} // namespace

RobotWorker::RobotWorker()
	: robot_(Robot::START),                   //
	  navigation_(Nav::OUTBOUND),             //
	  roller_(Roller::FORWARD),               //
	  rpi_cam_(new PanoramicCameraWorker(0)), //
	  serial_port_(new SerialPortWorker())    //
{
	home_goal_publisher_ =
		node_handle_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", queue_size);

	std::cout << "constructor\n" << std::endl;
}

RobotWorker::~RobotWorker() {
	if (th_operate_.joinable()) {
		th_operate_.join();
	}

	if (th_navigate_.joinable()) {
		th_navigate_.join();
	}

	if (th_capture_.joinable()) {
		th_capture_.join();
	}

	if (th_subscribe_to_topics_.joinable()) {
		th_subscribe_to_topics_.join();
	}

	if (th_publish_topics_.joinable()) {
		th_publish_topics_.join();
	}

	delete rpi_cam_;
	delete serial_port_;
}

void RobotWorker::Start() {
	th_operate_ = std::thread(&RobotWorker::Operate, this);
}

void RobotWorker::Operate() {
	initial_time = ros::Time::now();
	ros::Rate rate(operation_rate);

	while (robot_ != Robot::STOP) {
		current_time = ros::Time::now();

		switch (robot_) {
		case Robot::START:
			// wait for the arduino to be on
			if ((current_time - initial_time).toSec() > running_time) {
				// starts the processes
				rpi_cam_->Start();
				serial_port_->Start();
				robot_ = Robot::STARTED;
			}
			break;
		case Robot::STARTED:
			if ((current_time - initial_time).toSec() > running_time + ready_time) {
				robot_ = Robot::RUNNING;
				th_navigate_ = std::thread(&RobotWorker::Navigate, this);
				th_capture_ = std::thread(&RobotWorker::Capture, this);
				th_publish_topics_ = std::thread(&RobotWorker::PublishTopics, this);
				th_subscribe_to_topics_ = std::thread(&RobotWorker::SubscribeToTopics, this);
			}
			break;

		case Robot::RUNNING:
			if ((current_time - initial_time).toSec() > stop_time) {
				// this->Home();
				robot_ = Robot::STOP;
			}
			break;

		case Robot::STOP:
			serial_port_->SetNullControlMessage();
			break;
		}
		this->PrintRobot();

		rate.sleep();
	}
}

void RobotWorker::Navigate() {
	ros::Time open_time;
	ros::Rate rate(operation_rate);

	while (robot_ == Robot::RUNNING) {
		switch (navigation_) {
		case Nav::OUTBOUND:
			/// TODO send control
			if ((current_time - initial_time).toSec() > inbound_time) {
				this->Home();
				navigation_ = Nav::INBOUND;
				std::cout << "going home on time" << std::endl;
			}

			if (serial_port_->GetBottleNumber() >= max_bottle_number) {
				this->Home();
				navigation_ = Nav::INBOUND;
				std::cout << "going home max bottles" << std::endl;
			}
			break;

		case Nav::INBOUND:
			if (is_home) {
				open_time = ros::Time::now();
				serial_port_->SetGatePosition(true);
				navigation_ = Nav::OPEN;
			}
			break;

		case Nav::OPEN:
			if ((current_time - open_time).toSec() > door_dt) {
				serial_port_->SetGatePosition(false);
				serial_port_->SetBottleNumber(0);
				is_home = false;
				navigation_ = Nav::OUTBOUND;
			}
			break;
		}

		this->PrintNavig();
		rate.sleep();
	}
}

void RobotWorker::Capture() {
	static bool stuck = false;
	ros::Time stuck_time;
	ros::Rate rate(operation_rate);

	while (robot_ == Robot::RUNNING) {
		this->PrintRolle();
		switch (roller_) {
		case Roller::FORWARD:
			serial_port_->SetRollerState(1);
			if (!serial_port_->GetRollerEncoderState()) {
				if (!stuck) {
					stuck_time = ros::Time::now();
					stuck = true;

				} else {
					if ((current_time - stuck_time).toSec() > refraction_time) {
						stuck_time = ros::Time::now();
						roller_ = Roller::BACKWARD;
					}
				}
			}
			if (serial_port_->GetBottleNumber() >= max_bottle_number) {
				roller_ = Roller::STOP;
			}
			break;

		case Roller::BACKWARD:
			serial_port_->SetRollerState(2);
			if ((current_time - stuck_time).toSec() > roller_dt) {
				stuck = false;
				roller_ = Roller::FORWARD;
			}
			break;

		case Roller::STOP:
			serial_port_->SetRollerState(0);
			if (serial_port_->GetBottleNumber() < max_bottle_number) {
				roller_ = Roller::FORWARD;
			}
			break;
		}

		rate.sleep();
	}
}

void RobotWorker::PrintRobot() {
	static Robot previous = Robot::n;
	if (robot_ == previous) {
		return;
	}

	switch (robot_) {
	case Robot::START:
		std::cout << "robot :: START " << std::endl;
		break;

	case Robot::STARTED:
		std::cout << "robot :: STARTED " << std::endl;
		break;

	case Robot::RUNNING:
		std::cout << "robot :: RUNNING " << std::endl;
		break;

	case Robot::STOP:
		std::cout << "robot :: STOP " << std::endl;
		break;
	}
	previous = robot_;
}

void RobotWorker::PrintNavig() {
	static Nav previous = Nav::n;
	if (navigation_ == previous) {
		return;
	}
	switch (navigation_) {
	case Nav::OUTBOUND:
		std::cout << "navig :: OUTBOUND" << std::endl;
		break;

	case Nav::INBOUND:
		std::cout << "navig :: INBOUND" << std::endl;
		break;

	case Nav::OPEN:
		std::cout << "navig :: OPEN" << std::endl;
		break;
	}
	previous = navigation_;
}

void RobotWorker::PrintRolle() {
	static Roller previous = Roller::n;
	if (roller_ == previous) {
		return;
	}
	switch (roller_) {
	case Roller::FORWARD:
		std::cout << "rolle :: FORWARD" << std::endl;
		break;

	case Roller::BACKWARD:
		std::cout << "rolle :: BACKWARD" << std::endl;
		break;

	case Roller::STOP:
		std::cout << "rolle :: STOP" << std::endl;
		break;
	}
	previous = roller_;
}

void RobotWorker::Home() {
	std::cout << "trying to home" << std::endl;

	move_base_msgs::MoveBaseActionGoal home_goal_msg;

	ros::Time time = ros::Time::now();

	home_goal_msg.header.stamp = time;
	home_goal_msg.goal_id.stamp = time;
	home_goal_msg.goal_id.id = "home_goal_id";
	home_goal_msg.goal.target_pose.header.stamp = time;
	home_goal_msg.goal.target_pose.header.frame_id = "map";
	home_goal_msg.goal.target_pose.pose.position.x = 0.5;
	home_goal_msg.goal.target_pose.pose.position.y = 0.5;
	home_goal_msg.goal.target_pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion home_goal_quat = tf::createQuaternionMsgFromYaw(0.7853981633974483);
	home_goal_msg.goal.target_pose.pose.orientation = home_goal_quat;
	// home_goal_msg.goal.target_pose.pose.orientation.x = 0.0;
	// home_goal_msg.goal.target_pose.pose.orientation.y = 0.0;
	// home_goal_msg.goal.target_pose.pose.orientation.z = 0.7853981633974483;
	// home_goal_msg.goal.target_pose.pose.orientation.w = 1.0;

	home_goal_publisher_.publish(home_goal_msg);
}

void RobotWorker::PublishTopics() {}

void RobotWorker::SubscribeToTopics() {
	ros::Subscriber nav_status_subscriber =
		node_handle_.subscribe("move_base/status", nav_goal_queue_size, NavStatusCallback);
	ros::Rate rate(tp_rate);

	while (robot_ != Robot::STOP) {
		ros::spinOnce();
		rate.sleep();
	}
}

void NavStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &goal_status) {
	// std::cout << "nav status callback with id " << std::endl;
	for (int i = 0; i < goal_status->status_list.size(); ++i) {
		if (goal_status->status_list[i].goal_id.id == "home_goal_id") {
			// std::cout << "monitor home" << std::endl;
			if (goal_status->status_list[i].status == actionlib_msgs::GoalStatus::SUCCEEDED) {
				is_home = true;
				std::cout << "reached home" << std::endl;
			}
		}
	}
}