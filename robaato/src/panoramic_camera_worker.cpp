// C++ headers
#include <cmath>
#include <iostream>

// OpenCV headers
#include <opencv2/opencv.hpp>

// ROS headers
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <robot_state_publisher/robot_state_publisher.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>

// project headers
#include "definitions.h"
#include "panoramic_camera_worker.h"

using namespace std::chrono_literals;

namespace {
// h/s/v for hue, value, saturation
// lower case / upper case for min Max
// r/g/b/y for red, green, blue, yellow

// saturation
const uint16_t s = 50;
const uint16_t S = 255;

// value
const uint16_t v = 80;
const uint16_t V = 255;

// red
const uint16_t hr = 156;
const uint16_t HR = 180;

// green
const uint16_t hg = 20;
const uint16_t HG = 80;

// blue
const uint16_t hb = 85;
const uint16_t HB = 124;

// magneta
const uint16_t hm = 128;
const uint16_t HM = 145;

const uint16_t arena_width = 8;      //[m]
const uint16_t arena_height = 8;     //[m]
const uint16_t reachable_width = 8;  //[m]
const uint16_t reachable_height = 5; //[m]

// image metrics
// const uint16_t frame_width = 640;
const uint16_t frame_height = 480;
const cv::Point2f frame_center(335, 250);
const double outer_radius = 200;

const uint16_t magenta_pos_x = 0;
const uint16_t magenta_pos_y = 0;
const uint16_t green_pos_x = arena_width;
const uint16_t green_pos_y = arena_height;
const double cam_pano_rot_init = 4 * M_PI / 3;

} // namespace

PanoramicCameraWorker::PanoramicCameraWorker(uint64_t index)
	: CameraWorker(index) //
{
	std::cout << "pcw constructor" << std::endl;
}

PanoramicCameraWorker::~PanoramicCameraWorker() {
	std::cout << "pcw destructor" << std::endl;
}

void PanoramicCameraWorker::BroadcastTfs() {
	// odometry topic and odometry to base link tf
	ros::Publisher odom_publisher =
		node_handle_.advertise<nav_msgs::Odometry>("odom", queue_size); // odometry topic
	tf::TransformBroadcaster odom2base_link_bc; // (odom -> base_link) broadcaster

	double x = 0.5;
	double y = 0.5;
	double yaw = 0.0;
	double v_x = 0;
	double v_y = 0;
	double v_yaw = 0;

	ros::Time time_stamp;
	ros::Time current_time = ros::Time::now();
	ros::Time last_time = ros::Time::now();
	ros::Rate rate(tf_rate);

	while (!stop_) {
		last_time = current_time;
		current_time = ros::Time::now();

		double dt = (current_time - last_time).toSec();

		mu_position_.lock();
		if (dt != 0) {
			v_x = (pose_.x - x) / dt;
			v_y = (pose_.y - y) / dt;
			v_yaw = (pose_.yaw - yaw) / dt;
		}
		x = pose_.x;
		y = pose_.y;
		yaw = pose_.yaw;
		mu_position_.unlock();

		geometry_msgs::Quaternion odom_quat =
			tf::createQuaternionMsgFromYaw(yaw); // because odometry is 6DOF

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
		odom_msg.twist.twist.angular.z = v_yaw;

		odom_publisher.publish(odom_msg);

		rate.sleep();
	}
}

void PanoramicCameraWorker::ProcessFrame() {
	// don't anger higher powers
	std::this_thread::sleep_for(1ms);
	std::cout << "pcw process frame init" << std::endl; /// temp
	bool retval = true;                                 /// temp

	while (!stop_) {
		// get the latest frame
		mu_camera_feed_.lock();
		camera_feed_.retrieve(frame_);
		mu_camera_feed_.unlock();

		// update position from frame
		this->UpdatePoseFromFrame();

		++frame_index_;

		// sleep
		std::this_thread::sleep_for(20ms); // 50ms working
	}
}

void PanoramicCameraWorker::UpdatePoseFromFrame() {
	// img
	cv::Mat polar_frame;

	// convert from BGR to HSV
	// dynamic is still uint8 (0 - 255)
	// cv::cvtColor(srcImage, polar_frame, cv::COLOR_BGR2HSV);
	cv::cvtColor(frame_, polar_frame, cv::COLOR_BGR2HSV);

	// convert from polar to cartesian coordinates
	cv::Mat cart_frame;
	cv::linearPolar(polar_frame, cart_frame, frame_center, outer_radius, cv::INTER_LINEAR);

	// extract each LED stripe according to color in HSV space
	cv::Mat r_cart_mask;
	cv::Mat g_cart_mask;
	cv::Mat b_cart_mask;
	cv::Mat m_cart_mask;
	cv::inRange(cart_frame, cv::Scalar(hr, s, v), cv::Scalar(HR, S, V), r_cart_mask);
	cv::inRange(cart_frame, cv::Scalar(hg, s, v), cv::Scalar(HG, S, V), g_cart_mask);
	cv::inRange(cart_frame, cv::Scalar(hb, s, v), cv::Scalar(HB, S, V), b_cart_mask);
	cv::inRange(cart_frame, cv::Scalar(hm, s, v), cv::Scalar(HM, S, V), m_cart_mask);

	// horizontal projection to get a column vector
	uint8_t dim = 1;
	cv::Mat r_projection;
	cv::Mat g_projection;
	cv::Mat b_projection;
	cv::Mat m_projection;
	cv::reduce(r_cart_mask, r_projection, dim, cv::REDUCE_SUM, CV_32F);
	cv::reduce(g_cart_mask, g_projection, dim, cv::REDUCE_SUM, CV_32F);
	cv::reduce(b_cart_mask, b_projection, dim, cv::REDUCE_SUM, CV_32F);
	cv::reduce(m_cart_mask, m_projection, dim, cv::REDUCE_SUM, CV_32F);

	// max, only the x coord counts
	cv::Point r_pos;
	cv::Point g_pos;
	cv::Point b_pos;
	cv::Point m_pos;
	cv::minMaxLoc(r_projection, NULL, NULL, NULL, &r_pos);
	cv::minMaxLoc(g_projection, NULL, NULL, NULL, &g_pos);
	cv::minMaxLoc(b_projection, NULL, NULL, NULL, &b_pos);
	cv::minMaxLoc(m_projection, NULL, NULL, NULL, &m_pos);

	// normalise angles
	double r_angle = this->VectorIndex2RadAngle(r_pos.y);
	double g_angle = this->VectorIndex2RadAngle(g_pos.y);
	double b_angle = this->VectorIndex2RadAngle(b_pos.y);
	double m_angle = this->VectorIndex2RadAngle(m_pos.y);

	// find relative angles
	double angle_MPR = abs(m_angle - r_angle);
	double angle_RPG = abs(r_angle - g_angle);
	double angle_GPB = abs(g_angle - b_angle);
	double angle_BPM = abs(m_angle - b_angle);

	// convert the circular behaviour of cartesian
	AngleCircleConvert(&angle_MPR);
	AngleCircleConvert(&angle_RPG);
	AngleCircleConvert(&angle_GPB);
	AngleCircleConvert(&angle_BPM);

	double x = 0, y = 0, robot_angle = 0;

	Triangulate(&x, &y, &robot_angle, angle_MPR, angle_RPG, angle_GPB, angle_BPM, m_angle);

	static double x_prev = 1, y_prev = 1, robot_angle_prev = 0;
	double dist_from_prev = sqrt((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev));
	double angle_dif_from_prev = abs(robot_angle - robot_angle_prev);

	if (x < 0 || y < 0 || x > reachable_width || y > reachable_height) {
		std::cout << "error position outsode arena x: " << x << "y: " << y << std::endl;
	} else if (dist_from_prev > 2) {
		std::cout << "error too far from previous value x_prev: " << x << "y_prev: " << y
				  << std::endl;
	} else {

		/*
		//averaging over nb_data_mean datapoints:
		static double x_sum = {0};
		static double y_sum = {0};
		static double robot_angle_sum = {0};
		static uint8_t counter_data = 0;
		static uint8_t nb_data_mean = 4;

		if (counter_data<nb_data_mean){
			x_sum += x;
			y_sum += y;
			robot_angle_sum += robot_angle;
		} else{

			mu_position_.lock();
			pose_.x = (x+x_sum)/(nb_data_mean+1);
			pose_.y = (y+y_sum)/(nb_data_mean+1);
			pose_.yaw = (robot_angle+robot_angle_data)/(nb_data_mean+1);
			mu_position_.unlock();
		}

		*/

		std::cout << "x " << x << std::endl;
		std::cout << "y " << y << std::endl;
		std::cout << "robot_angle " << robot_angle << std::endl;

		// assign
		mu_position_.lock();
		pose_.x = x;
		pose_.y = y;
		pose_.yaw = robot_angle;
		mu_position_.unlock();

		// update pevious values
		x_prev = x;
		y_prev = y;
		robot_angle_prev = robot_angle;
	}
}

double PanoramicCameraWorker::VectorIndex2RadAngle(double angle) {
	return angle * 2 * M_PI / (double)frame_height;
}

// if relative angle is from last to first colours in cartisian image
void PanoramicCameraWorker::AngleCircleConvert(double *angle) {
	if (*angle > M_PI) {
		*angle = 2 * M_PI - *angle;
	}
}

void PanoramicCameraWorker::Triangulate(double *x, double *y, double *robot_angle, double MPR,
										double RPG, double GPB, double BPM, double m_angle) {
	/*
		return
			robot_angle
			[x,y] position from angles MPR, RPG, GPB, BPM
		M: magenta led
		R: red led
		G: green led
		B: Blue led
		P: PETBOT (robot) point with unknown coordinates

		M is (0,0)
		B (w,0)
		R (0,h)
		G (w,h)
	*/
	// choose triangle in which point P is found

	if ((MPR + BPM) > M_PI) { // deg
		// choose lower triangle
		double radius_mr = 0, orig_midway_dist_mr = 0, radius_mb = 0, orig_midway_dist_mb = 0;

		this->FindCircleOrigin(&radius_mr, &orig_midway_dist_mr, MPR, arena_height);
		this->FindCircleOrigin(&radius_mb, &orig_midway_dist_mb, BPM, arena_width);
		double Omr_x = magenta_pos_x + orig_midway_dist_mr;
		double Omr_y = magenta_pos_y + 0.5 * arena_height;

		double Omb_x = magenta_pos_x + 0.5 * arena_width;
		double Omb_y = magenta_pos_y + orig_midway_dist_mb;

		this->CirclesIntersection(x, y, Omr_x, Omr_y, radius_mr, Omb_x, Omb_y, radius_mb,
								  magenta_pos_x, magenta_pos_y);
	} else {
		// choose upper triangle
		double radius_gb = 0, orig_midway_dist_gb = 0, radius_gr = 0, orig_midway_dist_gr = 0;
		this->FindCircleOrigin(&radius_gb, &orig_midway_dist_gb, GPB, arena_height);
		this->FindCircleOrigin(&radius_gr, &orig_midway_dist_gr, RPG, arena_width);

		double Ogb_x = green_pos_x - orig_midway_dist_gb;
		double Ogb_y = green_pos_y - 0.5 * arena_height;

		double Ogr_x = green_pos_x - 0.5 * arena_width;
		double Ogr_y = green_pos_y - orig_midway_dist_gr;

		this->CirclesIntersection(x, y, Ogb_x, Ogb_y, radius_gb, Ogr_x, Ogr_y, radius_gr,
								  green_pos_x, green_pos_y);
	}

	*robot_angle =
		atan2(((magenta_pos_y - *y)), (magenta_pos_x - *x)) - m_angle + cam_pano_rot_init;
	*robot_angle = *robot_angle + M_PI / 2; // translate to arena coordinates
	if (*robot_angle > M_PI) {
		*robot_angle = *robot_angle - 2 * M_PI;
	}

	// translate to center of rotation
	*x = *x - 0.18475 * cos(*robot_angle);
	*y = *y - 0.18475 * sin(*robot_angle);
}

void PanoramicCameraWorker::FindCircleOrigin(double *radius, double *orig_midway_dist, double alpha,
											 double dist) {
	/* find circle properties
	   radius: of circle
	   orig_midway_dist: smallest distance between origin and arena border
	   inputs:
	   alpha: angle between two corners of arena
	   dist: distance between two corners of arena
	*/
	// find origin of circle
	if (alpha == M_PI / 2) {
		// origin on arena border
		*radius = dist;
		*orig_midway_dist = 0;
	} else {
		double corners_origin_angle = 0;
		double origin_dir = 0;
		if (alpha > M_PI / 2) {
			// origin outside arena
			corners_origin_angle = 2 * M_PI - 2 * alpha;
			origin_dir = -1;
		} else {
			// origin inside arena
			corners_origin_angle = 2 * alpha;
			origin_dir = 1;
		}

		double corner_midway_angle = 0.5 * corners_origin_angle;

		if (sin(corner_midway_angle) != 0)
			*radius = abs((0.5 * dist) / sin(corner_midway_angle));
		else
			std::cout << "sin angle error" << std::endl;

		if (tan(corner_midway_angle) != 0)
			*orig_midway_dist = origin_dir * abs((0.5 * dist) / tan(corner_midway_angle));
		else
			std::cout << "tangent angle error" << std::endl;
	}
}

void PanoramicCameraWorker::CirclesIntersection(double *x, double *y, double O1_x, double O1_y,
												double R1, double O2_x, double O2_y, double R2,
												double corner_x, double corner_y) {
	/* find two possible itersection points of two circles
	 O1: origin circle 1
	 O2: origin circle 2
	 R: radius

	 returns coordinates of intersection which has only positive coordinates.
	 If one of the coordinates is negative, hence the point is outside the arena. Since robot always
	 insode arena, take the furthest point from corner.
	*/
	// intermediate variables in calculations
	double a = ((pow(R1, 2) - pow(R2, 2)) - (pow(O1_x, 2) - pow(O2_x, 2)) -
				(pow(O1_y, 2) - pow(O2_y, 2))) /
			   (2 * (O2_x - O1_x));
	double b = (O2_y - O1_y) / (O2_x - O1_x);

	double c = (1 + pow(b, 2));
	double d = (2 * b * O1_x - 2 * O1_y - 2 * b * a);
	double e = (pow(a, 2) - 2 * a * O1_x + pow(O1_x, 2) + pow(O1_y, 2) - pow(R1, 2));

	double det = pow(d, 2) - 4 * c * e;

	double x1 = 0;
	double y1 = 0;
	double x2 = 0;
	double y2 = 0;

	if (det >= 0) {
		y1 = (-d + sqrt(det)) / (2 * c);
		x1 = a - b * y1;

		y2 = (-d - sqrt(det)) / (2 * c);
		x2 = a - b * y2;
	} else
		std::cout << "det negative" << std::endl;

	// choose pair of coords which is further away from the corner (one pair almost on the corner
	// and the other is inside the arena
	double dist2corner_1 = pow((x1 - corner_x), 2) + pow((y1 - corner_y), 2);
	double dist2corner_2 = pow((x2 - corner_x), 2) + pow((y2 - corner_y), 2);
	if (dist2corner_1 > dist2corner_2) {
		*x = x1;
		*y = y1;
	} else {
		*x = x2;
		*y = y2;
	}
}

void PanoramicCameraWorker::PublishTopics() {
	ros::Time time_stamp;
	ros::Rate rate(tp_rate);

	while (!stop_) {
		rate.sleep();
	}
}
