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
const double f = 8;
geometry_msgs::Twist velocity_command;
double motor_command_l = 0;
double motor_command_r = 0;
double omega_wheel_l = 0;
double omega_wheel_r = 0;
double v_lin = 0;
double v_ang = 0;
const double bl[TOF_SENSOR_NUM] = {-1 * f, -1 * f, 1 * f, -1 * f, 1 * f, 1 * f};
const double br[TOF_SENSOR_NUM] = {-1 * f, 1 * f, -1 * f, 1 * f, -1 * f, 1 * f};
double cbl = 0;
double cbr = 0;

// range value min-max
float min_range = 0.02; // 2 cm
float max_range = 1.00; // 50 cm

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
			  << "v roll " << sensor_message_.imu[V_ROLL] << std::endl
			  << "v pitch " << sensor_message_.imu[V_PITCH] << std::endl
			  << "v yaw " << sensor_message_.imu[V_YAW] << std::endl
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

void SerialPortWorker::WriteNullControlMessage() {
	control_message_.gate_position = false;
	mu_velocity_command.lock();
	control_message_.left_wheel_direction = 0;
	control_message_.right_wheel_direction = 0;
	control_message_.left_wheel_speed = 0;
	control_message_.right_wheel_speed = 0;
	mu_velocity_command.unlock();
	control_message_.roller_state = 0;

	write(serial_port_, (char *)&control_message_, sizeof(control_message_));
}

void SerialPortWorker::WriteControlMessage() {

	// if (sensor_message_.tof_sensors[0] < max_range || sensor_message_.tof_sensors[1] < max_range
	// || 	sensor_message_.tof_sensors[2] < max_range || sensor_message_.tof_sensors[3] < max_range
	// ||
	//	sensor_message_.tof_sensors[4] < max_range || sensor_message_.tof_sensors[5] < max_range) {

	//	std::cout << "NOT RESET\n" << std::endl;
	//	//this->DisplaySensorReadings();
	//	for (size_t i = 0; i < TOF_SENSOR_NUM; ++i) {
	//		if (sensor_message_.tof_sensors[i] != 0) {
	//			cbl += bl[i] / sensor_message_.tof_sensors[i];
	//			cbr += br[i] / sensor_message_.tof_sensors[i];
	//		}
	//	}
	//} else {
	//	std::cout << "RESET\n" << std::endl;
	//	//this->DisplaySensorReadings();
	//	cbl = 0;
	//	cbr = 0;
	//	// motor_command_l = 0;
	//	// motor_command_r = 0;
	//}

	// motor_command_l += cbl;
	// motor_command_r += cbr;
	// motor_command_l = std::clamp(motor_command_l, -3000., 3000.);
	// motor_command_r = std::clamp(motor_command_r, -3000., 3000.);

	// std::cout << "lcm " << motor_command_l << std::endl
	//		  << "rmc " << motor_command_r << std::endl
	//		  << "lef buff " << cbl << std::endl
	//		  << "rig buff " << cbr << "\n"
	//		  << std::endl;

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

	ros::Time start_time = ros::Time::now();
	ros::Time current_time = start_time;

	while ((current_time - start_time).toSec() < 20) {
		current_time = ros::Time::now();
		// write to serial port
		this->WriteNullControlMessage();

		// read from serial port
		this->ReadSensorMessage();
	}
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
	this->WriteNullControlMessage();

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

	ros::Time current_time = ros::Time::now();
	ros::Time last_time = ros::Time::now();
	ros::Rate rate(tf_rate);

	while (!stop_) {
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
