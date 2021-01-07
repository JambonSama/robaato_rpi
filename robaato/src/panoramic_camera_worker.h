#pragma once

// ROS headers
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

// project headers
#include "camera_worker.h"

struct Pose {
	double x = 0;
	double y = 0;
	double yaw = 0;
};

// struct Position {
// 	double x = 0;
// 	double y = 0;
// };

class PanoramicCameraWorker : public CameraWorker {
protected:
	// BASE CLASS MEMBERS
	Pose pose_; // [x, y, yaw] robot pose
	ros::NodeHandle node_handle_;

	// MUTEX CLASS MEMBERS
	std::mutex mu_pose_; // mutex to access the robot position

	// CLASS METHODS
	// processes the frame from retrieval to robot position
	void ProcessFrame() override;
	// triangulates position from LED angles [rad]
	void Triangulate(double *x, double *y, double *robot_angle, double MPR, double RPG, double GPB,
					 double BPM, double m_angle);
	// produces the LED stripes angle in radian from the position within the projected vector
	double VectorIndex2RadAngle(double angle);
	// if relative angle is from last to first colours in cartisian image
	void AngleCircleConvert(double *angle);
	// find circle properties for triangulation algorithm
	void FindCircleOrigin(double *radius, double *orig_midway_dist, double alpha, double dist);
	// find two possible itersection points of two circles: for triangulation algorithm
	void CirclesIntersection(double *x, double *y, double O1_x, double O1_y, double R1, double O2_x,
							 double O2_y, double R2, double corner_x, double corner_y);
	// updates the robot position from the frame
	void UpdatePoseFromFrame();

	void BroadcastTfs();
	void PublishTopics();

public:
	// constructs the camera worker from the index of the camera on the device
	// see v4l2-ctl --list-devices
	PanoramicCameraWorker(uint64_t index);
	virtual ~PanoramicCameraWorker();
};
