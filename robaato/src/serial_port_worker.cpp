// C++ library headers
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iostream>

// ROS headers
#include <kdl_parser/kdl_parser.hpp>
#include <nav_msgs/Odometry.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>

// Linux headers
#include <errno.h>   // error integer and strerror() function
#include <fcntl.h>   // contains file controls like O_RDWR
#include <termios.h> // contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

// project headers
#include "definitions.h"
#include "serial_port_worker.h"

using namespace std::chrono_literals;

namespace {
const size_t tof_sensor_number = 6;

// FM FR FL SR SL BM
const std::string link_names[tof_sensor_number] = {"tof_fm", "tof_fr", "tof_fl",
												   "tof_sr", "tof_sl", "tof_bm"};
const std::string link_suffix = "_link";

// robot dimensions
const double R = 0.0350;
const double L = 0.31400 + 2 * 0.00850 + 0.0240;

// gear reduction
const double gear_reduction = 172;

// motor command
geometry_msgs::Twist velocity_command;
double motor_command_l;
double motor_command_r;
double omega_wheel_l;
double omega_wheel_r;
double v_lin;
double v_ang;

// range value min-max
float min_range = 0.02; // 2 cm
float max_range = 4.00; // 4 m

// mutexes
std::mutex mu_velocity_command;
std::mutex mu_motor_command;
std::mutex mu_omega_wheel;
} // namespace

SerialPortWorker::SerialPortWorker()
	: serial_port_(open("/dev/ttyACM0", O_RDWR)) // open the serial port (currently set to an
												 // standard FTDI USB-UART cable type device)
{}

SerialPortWorker::~SerialPortWorker() {
	std::cout << "spw destructor" << std::endl; /// temp

	stop_ = true;

	if (th_read_write_serial_port_.joinable()) {
		th_read_write_serial_port_.join();
	}
	std::cout << "spw destructor read/write serial port" << std::endl; /// temp

	if (th_update_velocity_command_.joinable()) {
		th_update_velocity_command_.join();
	}
	std::cout << "spw destructor get command" << std::endl; /// temp

	if (th_broadcast_tfs_.joinable()) {
		th_broadcast_tfs_.join();
	}

	if (th_publish_topics_.joinable()) {
		th_publish_topics_.join();
	}
}

void SerialPortWorker::ConfigureSerialPort() {
	// create new termios struc, we call it 'tty' for convention
	struct termios tty;

	// read in existing settings, and handle any error
	if (tcgetattr(serial_port_, &tty) != 0) {
		std::cout << "ò_ó tcgetattr serial port\n";
		return;
	}

	tty.c_cflag &= ~PARENB;        // clear parity bit, disabling parity (most common)
	tty.c_cflag &= ~CSTOPB;        // clear stop field, only one stop bit used in communication
	tty.c_cflag &= ~CSIZE;         // clear all bits that set the data size
	tty.c_cflag |= CS8;            // 8 bits per byte (most common)
	tty.c_cflag &= ~CRTSCTS;       // disable RTS/CTS hardware flow control (most common)
	tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines (CLOCAL = 1)

	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;                   // disable echo
	tty.c_lflag &= ~ECHOE;                  // disable erasure
	tty.c_lflag &= ~ECHONL;                 // disable new-line echo
	tty.c_lflag &= ~ISIG;                   // disable interpretation of INTR, QUIT and SUSP
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
					 ICRNL); // disable any special handling of received bytes

	tty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes (e.g. newline chars)
	tty.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return/line feed

	tty.c_cc[VTIME] = 10; // wait for up to x deciseconds, returning as soon as any data is received
	tty.c_cc[VMIN] = sizeof(SensorMessage); // how much to read

	// set in/out baud rate to be 9600
	cfsetispeed(&tty, B115200);
	cfsetospeed(&tty, B115200);

	// save tty settings, also checking for error
	if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
		std::cout << "ò_ó tcsetattr serial port\n";
		return;
	}
}

void SerialPortWorker::DisplayControlCommand() {
	std::cout << "v_lin " << v_lin << std::endl
			  << "v_ang " << v_ang << std::endl
			  << "omega_wheel_l " << omega_wheel_l << std::endl
			  << "omega_wheel_r " << omega_wheel_r << std::endl
			  << "motor_command_l " << motor_command_l << std::endl
			  << "motor_command_r " << motor_command_r << "\n"
			  << std::endl;
}

void SerialPortWorker::DisplaySensorReadings() {
	std::cout << "ax " << sensor_message_.imu[A_X] << std::endl
			  << "ay " << sensor_message_.imu[A_Y] << std::endl
			  << "az " << sensor_message_.imu[A_Z] << std::endl
			  << "vel roll " << sensor_message_.imu[V_ROLL] << std::endl
			  << "vel pitch " << sensor_message_.imu[V_PITCH] << std::endl
			  << "vel yaw " << sensor_message_.imu[V_YAW] << std::endl
			  << "tof fm " << sensor_message_.tof_sensors[FM] << std::endl
			  << "tof fr " << sensor_message_.tof_sensors[FR] << std::endl
			  << "tof fl " << sensor_message_.tof_sensors[FL] << std::endl
			  << "tof sr " << sensor_message_.tof_sensors[SR] << std::endl
			  << "tof sl " << sensor_message_.tof_sensors[SL] << std::endl
			  << "tof bm " << sensor_message_.tof_sensors[BM] << std::endl
			  << "bottle sensor " << sensor_message_.bottle_sensor << std::endl
			  << "roller encoder " << sensor_message_.roller_encoder_state << "\n"
			  << std::endl;
}

void SerialPortWorker::WriteControlMessage() {
	control_message_.gate_position = false;
	mu_velocity_command.lock();
	control_message_.left_wheel_direction = motor_command_l > 0;
	control_message_.right_wheel_direction = motor_command_r > 0;
	control_message_.left_wheel_speed = std::fabs(motor_command_l);
	control_message_.right_wheel_speed = std::fabs(motor_command_r);
	mu_velocity_command.unlock();
	control_message_.roller_state = 0;

	write(serial_port_, (char *)&control_message_, sizeof(control_message_));
}

void SerialPortWorker::ReadSensorMessage() {
	int num_bytes = read(serial_port_, (char *)&sensor_message_, sizeof(sensor_message_));

	// check for error (n negative)
	if (num_bytes < 0) {
		std::cout << "ò_ó reading serial port\n";
		return;
	}
}

void SerialPortWorker::UpdateVelocityCommand() {
	ros::Subscriber command_suscriber =
		node_handle_.subscribe("cmd_vel", queue_size, VelocityCmdCallback);
	ros::Rate rate(tp_rate); // rate in [Hz]
	while (!stop_) {
		ros::spinOnce();
		rate.sleep();
	}
}

void SerialPortWorker::ReadWriteSerialPort() {
	while (!stop_) {
		// write to serial port
		this->WriteControlMessage();

		// read from serial port
		this->ReadSensorMessage();

		// sensor cout display (for debug)
		// this->DisplayControlCommand();
		// this->DisplaySensorReadings();
	}

	// write to serial port to stop motors
	control_message_.gate_position = false;
	mu_velocity_command.lock();
	control_message_.left_wheel_direction = true;
	control_message_.right_wheel_direction = true;
	control_message_.left_wheel_speed = 0;
	control_message_.right_wheel_speed = 0;
	mu_velocity_command.unlock();
	control_message_.roller_state = 0;

	write(serial_port_, (char *)&control_message_, sizeof(control_message_));

	close(serial_port_);

	std::cout << "spw: asked for motor stop" << std::endl;
}

void SerialPortWorker::Start() {
	this->ConfigureSerialPort();

	th_read_write_serial_port_ = std::thread(&SerialPortWorker::ReadWriteSerialPort, this);
	th_update_velocity_command_ = std::thread(&SerialPortWorker::UpdateVelocityCommand, this);
	th_broadcast_tfs_ = std::thread(&SerialPortWorker::BroadcastTfs, this);
	th_publish_topics_ = std::thread(&SerialPortWorker::PublishTopics, this);
}

void SerialPortWorker::BroadcastTfs() {
	// base link to robot tf
	KDL::Tree base_link2robot_tf; // (base_link -> chassis -> (tof tf + imu + cam)) transform

	if (!kdl_parser::treeFromFile(
			"/home/pi/robaato_ck_ws/src/robaato_description/urdf/robaato.urdf",
			base_link2robot_tf)) {
		ROS_ERROR("Failed to construct kdl tree");
	}
	robot_state_publisher::RobotStatePublisher base_link2robot_tf_bc(
		base_link2robot_tf); // (base_link -> robot) broadcaster

	// odometry topic and odometry to base link tf
	ros::Publisher odom_publisher =
		node_handle_.advertise<nav_msgs::Odometry>("odom", queue_size); // odometry topic
	tf::TransformBroadcaster odom2base_link_bc; // (odom -> base_link) broadcaster

	double x = 0.5;
	double y = 0.5;
	double th = 0.0;

	ros::Time current_time = ros::Time::now();
	ros::Time last_time = ros::Time::now();
	ros::Rate rate(tf_rate);

	while (!stop_) {
		// compute odometry
		double v_x = v_lin * cos(th);
		double v_y = v_lin * sin(th);

		last_time = current_time;
		current_time = ros::Time::now();

		double dt = (current_time - last_time).toSec();
		double delta_x = v_x * dt;
		double delta_y = v_y * dt;
		double delta_th = v_ang * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;
		geometry_msgs::Quaternion odom_quat =
			tf::createQuaternionMsgFromYaw(th); // because odometry is 6DOF

		// odometry to base link tf
		geometry_msgs::TransformStamped odom2base_link_tf; // (odom -> base_link) transform
		odom2base_link_tf.header.stamp = current_time;
		odom2base_link_tf.header.frame_id = "odom";
		odom2base_link_tf.child_frame_id = "base_link";

		odom2base_link_tf.transform.translation.x = x;
		odom2base_link_tf.transform.translation.y = y;
		odom2base_link_tf.transform.translation.z = 0.0;
		odom2base_link_tf.transform.rotation = odom_quat;

		// odometry to base link tf broadcasting
		odom2base_link_bc.sendTransform(odom2base_link_tf);

		// odometry topic
		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_link";

		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;
		odom_msg.twist.twist.linear.x = v_x;
		odom_msg.twist.twist.linear.y = v_y;
		odom_msg.twist.twist.linear.z = 0;
		odom_msg.twist.twist.angular.x = 0;
		odom_msg.twist.twist.angular.y = 0;
		odom_msg.twist.twist.angular.z = v_ang;

		odom_publisher.publish(odom_msg);

		// publish (base link -> robot) tf
		base_link2robot_tf_bc.publishFixedTransforms("");

		rate.sleep();
	}
}

void SerialPortWorker::PublishTopics() {
	// tof (FM FR FL SR SL BM)
	ros::Publisher tof_publisher[tof_sensor_number];
	sensor_msgs::Range tof_msg[tof_sensor_number];

	for (uint8_t i = 0; i < tof_sensor_number; ++i) {
		tof_publisher[i] = node_handle_.advertise<sensor_msgs::Range>(link_names[i], queue_size);
		tof_msg[i].header.frame_id = link_names[i] + link_suffix;
		tof_msg[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
		tof_msg[i].field_of_view = 0.5235987755982989; // 30 deg = 0.52 rad
		tof_msg[i].min_range = min_range;
		tof_msg[i].max_range = max_range;
	}

	// // imu
	// ros::Publisher imu_publisher;
	// imu_publisher = node_handle_.advertise<sensor_msgs::Imu>("localization/imu", queue_size);
	// sensor_msgs::Imu imu_msg;
	// imu_msg.header.frame_id = "imu_link";
	// imu_msg.orientation_covariance = {-1, 0, 0, 0, 0,
	// 								  0,  0, 0, 0}; // because it doesn't give angular position
	// imu_msg.angular_velocity_covariance = {0.000003, 0, 0, 0, 0.000035, 0, 0, 0, 0.00137};
	// imu_msg.linear_acceleration_covariance = {0.0005, 0, 0, 0, 0.00016, 0, 0, 0, 0.00056};

	// time
	ros::Time time_stamp;
	ros::Rate rate(tp_rate);

	while (!stop_) {
		// time update
		time_stamp = ros::Time::now();

		// tof
		for (uint8_t i = 0; i < TOF_SENSOR_NUM; ++i) {
			tof_msg[i].header.stamp = time_stamp;
			tof_msg[i].range = std::clamp(sensor_message_.tof_sensors[i], min_range, max_range);
			tof_publisher[i].publish(tof_msg[i]);
		}

		// imu_msg.header.stamp = time_stamp;
		// imu_msg.angular_velocity.x = 0;       // sensor_message_.imu[V_ROLL];
		// imu_msg.angular_velocity.y = 0;       // sensor_message_.imu[V_PITCH];
		// imu_msg.angular_velocity.z = 0;       // sensor_message_.imu[V_YAW];
		// imu_msg.linear_acceleration.x = 0;    // sensor_message_.imu[A_X];
		// imu_msg.linear_acceleration.y = 0;    // sensor_message_.imu[A_Y];
		// imu_msg.linear_acceleration.z = 9.81; // sensor_message_.imu[A_Z];
		// imu_publisher.publish(imu_msg);

		rate.sleep();
	}
}

void VelocityCmdCallback(const geometry_msgs::Twist::ConstPtr &cmd_vel) {
	mu_velocity_command.lock();
	velocity_command = *cmd_vel;
	mu_velocity_command.unlock();

	v_lin = velocity_command.linear.x;  // SI [m/s]
	v_ang = velocity_command.angular.z; // SI [rad/s]

	mu_omega_wheel.lock();
	omega_wheel_l = (2 * v_lin - v_ang * L) / (2 * R); // SI [rad/s]
	omega_wheel_r = (2 * v_lin + v_ang * L) / (2 * R); // SI [rad/s]
	mu_omega_wheel.unlock();

	mu_motor_command.lock();
	motor_command_l = omega_wheel_l * 30 / M_PI * gear_reduction; // rpm
	motor_command_r = omega_wheel_r * 30 / M_PI * gear_reduction; // rpm
	mu_motor_command.unlock();
}
