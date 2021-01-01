#include "serial_port_worker.h"

// C++ library headers
#include <iostream>

// C library headers
//#include <stdio.h>
#include <cstring>

// Linux headers
#include <errno.h>   // error integer and strerror() function
#include <fcntl.h>   // contains file controls like O_RDWR
#include <termios.h> // contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()

using namespace std::chrono_literals;

namespace {
// queue size for published values
const uint32_t queue_size = 50;
const size_t tof_sensor_number = 6;

// FM FR FL SR SL BM
const std::string link_names[tof_sensor_number] = {"tof_fm","tof_fr","tof_fl","tof_sr","tof_sl","tof_bm"};
const std::string link_suffix = "_link";
const std::string frame_suffix = "_frame";

// robot dimensions
const double R = 0.0350;
const double L = 0.31400+2*0.00850+0.0240;

// motor command
double leftMotorCommand;
double rightMotorCommand;

std::mutex mu_command;
}

SerialPortWorker::SerialPortWorker() {}

SerialPortWorker::~SerialPortWorker() {
    std::cout << "spw destructor" << std::endl; /// temp

    stop_ = true;

    if (th_read_write_serial_port_.joinable()) {
        th_read_write_serial_port_.join();
    }
    std::cout << "spw destructor read/write serial port" << std::endl; /// temp

    if (th_get_command_.joinable()) {
        th_get_command_.join();
    }
    std::cout << "spw destructor get command" << std::endl; /// temp
}

void SerialPortWorker::ReadWriteSerialPort() {
    // open the serial port
    // change device path as needed (currently set to an standard FTDI
    // USB-UART cable type device)
    int serial_port = open("/dev/ttyACM0", O_RDWR);

    // create new termios struc, we call it 'tty' for convention
    struct termios tty;

    // read in existing settings, and handle any error
    if (tcgetattr(serial_port, &tty) != 0) {
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
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        std::cout << "ò_ó tcsetattr serial port\n";
        return;
    }

    // FM FR FL SR SL BM
    ros::Publisher tof_publisher[tof_sensor_number];
    sensor_msgs::Range tof_range[tof_sensor_number];

    for (uint8_t i=0; i<tof_sensor_number; ++i){
        // publisher
        tof_publisher[i] = node_handle_.advertise<sensor_msgs::Range>(link_names[i]+frame_suffix, queue_size);
        // header
        tof_range[i].header.frame_id = link_names[i]+link_suffix;
        // rest of message
        tof_range[i].radiation_type = sensor_msgs::Range::ULTRASOUND;
        tof_range[i].field_of_view = 0.5235987755982989; // 30 deg = 0.52 rad
        tof_range[i].min_range = 0.02; // 2 cm
        tof_range[i].max_range = 4.00; // 4 m
    }

    ros::Publisher odom_publisher = node_handle_.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    while (!stop_) {

        // write to serial port
        control_message_.gate_position = false;
        mu_command.lock();
        control_message_.left_wheel_direction = leftMotorCommand>0;
        control_message_.right_wheel_direction = rightMotorCommand>0;
        control_message_.left_wheel_speed = std::fabs(leftMotorCommand);
        control_message_.right_wheel_speed = std::fabs(rightMotorCommand);
        mu_command.unlock();
        control_message_.roller_state = 0;

        write(serial_port, (char *)&control_message_, sizeof(control_message_));

        // read bytes
        // the behaviour of read() (e.g. does it block?,
        // how long does it block for?) depends on the configuration
        // settings above, specifically VMIN and VTIME
        int num_bytes = read(serial_port, (char *)&sensor_message_, sizeof(sensor_message_));

        // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1
        // to signal an error.
        if (num_bytes < 0) {
            std::cout << "ò_ó reading serial port\n";
            return;
        }

        std::cout 
        << "ax " << sensor_message_.ax << std::endl
        << "ay " << sensor_message_.ay << std::endl
        << "az " << sensor_message_.az << std::endl
        << "yaw " << sensor_message_.yaw << std::endl
        << "pitch " << sensor_message_.pitch << std::endl
        << "roll " << sensor_message_.roll << std::endl
        << "tof fm " << sensor_message_.tof_sensors[0] << std::endl
        << "tof fr " << sensor_message_.tof_sensors[1] << std::endl
        << "tof fl " << sensor_message_.tof_sensors[2] << std::endl
        << "tof sr " << sensor_message_.tof_sensors[3] << std::endl
        << "tof sl " << sensor_message_.tof_sensors[4] << std::endl
        << "tof bm " << sensor_message_.tof_sensors[5] << std::endl
        << "bottle sensor " << sensor_message_.bottle_sensor << std::endl
        << "roller encoder" << sensor_message_.roller_encoder_state << "\n" << std::endl;

        for (uint8_t i=0; i<TOF_SENSOR_NUM; ++i){
            // header
            tof_range[i].header.stamp = ros::Time::now();
            // rest of message
            tof_range[i].range = sensor_message_.tof_sensors[i];
            // publication
            tof_publisher[i].publish(tof_range[i]);
        }

        double omega_r = (control_message_.left_wheel_speed)*(control_message_.left_wheel_direction?1:-1)*M_PI/30;
        double omega_l = (control_message_.right_wheel_speed)*(control_message_.right_wheel_direction?1:-1)*M_PI/30;
        double v_lin = (omega_r + omega_l)*R/2;
        double v_ang = (omega_r - omega_l)*R/L;
        double v_x = v_lin * cos(th);
        double v_y = v_lin * sin(th);

        current_time = ros::Time::now();

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time).toSec();
        double delta_x = v_x * dt;
        double delta_y = v_y * dt;
        double delta_th = v_ang * dt;

        x += delta_x;
        y += delta_y;
        th += delta_th;

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = v_x;
        odom.twist.twist.linear.y = v_y;
        odom.twist.twist.angular.z = v_ang;

        //publish the message
        odom_publisher.publish(odom);
    }

    // write to serial port
    control_message_.left_wheel_direction = true;
    control_message_.right_wheel_direction = true;
    control_message_.left_wheel_speed = 0;
    control_message_.right_wheel_speed = 0;

    write(serial_port, (char *)&control_message_, sizeof(control_message_));

    std::cout << "spw: asked for motor stop" << std::endl;

    close(serial_port);
}

void SerialPortWorker::GetCommand(){
    ros::Subscriber command_suscriber = node_handle_.subscribe("cmd_vel", queue_size, CmdCallback);
    while(!stop_){
        ros::spinOnce();
        std::this_thread::sleep_for(3ms);
    }
}

void SerialPortWorker::Start() {
    th_read_write_serial_port_ = std::thread(&SerialPortWorker::ReadWriteSerialPort, this);
    th_get_command_ = std::thread(&SerialPortWorker::GetCommand, this);
}

void CmdCallback(const geometry_msgs::Twist::ConstPtr& velocity_command){
    double v_t = std::sqrt(velocity_command->linear.x*velocity_command->linear.x + velocity_command->linear.y*velocity_command->linear.y);

    double omega_t = velocity_command->angular.z;
    double omega_r = (2*v_t + omega_t*L) / (2*R); // SI
    double omega_l = (2*v_t - omega_t*L) / (2*R); // SI

    mu_command.lock();
    leftMotorCommand = omega_l*30/M_PI;
    rightMotorCommand = omega_r*30/M_PI;
    mu_command.unlock();
}
