// C++ library headers
#include <cstring>
#include <iostream>

// ROS headers
#include <actionlib_msgs/GoalID.h>
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
uint64_t current_goal_index = 0;
const Pose nav_goals[3] = {{4., 1., 0.}, {4., 4., 1.57}, {1., 4., 3.14}};
bool nav_goal_publish = true;
} // namespace

RobotWorker::RobotWorker()
	: robot_(Robot::START),                   //
	  navigation_(Nav::OUTBOUND),             //
	  roller_(Roller::FORWARD),               //
	  rpi_cam_(new PanoramicCameraWorker(0)), //
	  serial_port_(new SerialPortWorker())    //
{
	nav_goal_publisher_ =
		node_handle_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", queue_size);
	goal_id_publisher_ =
		node_handle_.advertise<actionlib_msgs::GoalID>("move_base/cancel", queue_size);

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
			if (nav_goal_publish) {
				move_base_msgs::MoveBaseActionGoal current_goal =
					Pose2Goal(nav_goals[current_goal_index], "nav_goal_id");
				nav_goal_publisher_.publish(current_goal);
				nav_goal_publish = false;
			}
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
				nav_goal_publish = true;
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

	actionlib_msgs::GoalID goal_id_msg;
	goal_id_msg.stamp = ros::Time::now();
	goal_id_msg.id = "nav_goal_id";

	goal_id_publisher_.publish(goal_id_msg);

	move_base_msgs::MoveBaseActionGoal home_goal_msg =
		Pose2Goal({0.5, 0.5, 0.7853981633974483}, "home_goal_id");

	nav_goal_publisher_.publish(home_goal_msg);
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
		if (goal_status->status_list[i].goal_id.id == "nav_goal_id") {
			if (goal_status->status_list[i].status == actionlib_msgs::GoalStatus::SUCCEEDED) {
				++current_goal_index;
				nav_goal_publish = true;
			}
		}
	}
}

move_base_msgs::MoveBaseActionGoal Pose2Goal(Pose pose, std::string id) {
	move_base_msgs::MoveBaseActionGoal goal_msg;

	ros::Time time = ros::Time::now();

	geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose.yaw);
	goal_msg.header.stamp = time;
	goal_msg.goal_id.stamp = time;
	goal_msg.goal_id.id = id;
	goal_msg.goal.target_pose.header.stamp = time;
	goal_msg.goal.target_pose.header.frame_id = "map";
	goal_msg.goal.target_pose.pose.position.x = pose.x;
	goal_msg.goal.target_pose.pose.position.y = pose.y;
	goal_msg.goal.target_pose.pose.position.z = 0.0;
	goal_msg.goal.target_pose.pose.orientation = goal_quat;

	return goal_msg;
}
