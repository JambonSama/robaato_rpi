// general c++ headers
//#include <iostream> // basic input output stream manipulations

// ros headers
#include <ros/ros.h>

// project headers
#include "robot_worker.h"
//#include "panoramic_camera_worker.h"
//#include "serial_port_worker.h"

int main(int argc, char **argv) {
	// ros init
	ros::init(argc, argv, "robaato");

	// robot worker
	RobotWorker robot;
	robot.Start();

	// // objects inits
	// CameraWorker *rpi_cam = new PanoramicCameraWorker(0);
	// SerialPortWorker *serial_port = new SerialPortWorker();

	//// thread starts
	// rpi_cam->Start();
	// serial_port->Start();

	// // get out of program
	// std::cin.ignore();

	// // object destruction
	// delete rpi_cam;
	// delete serial_port;

	return 0;
}
