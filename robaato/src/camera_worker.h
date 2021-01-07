#pragma once

#include <cstdint>
#include <mutex>
#include <thread>

#include <opencv2/core.hpp>    // basic ocv structs (mat, scalar...)
#include <opencv2/highgui.hpp> // imread / imwrite
#include <opencv2/imgproc.hpp> // HSV
#include <opencv2/videoio.hpp> // video input output

#include <iostream> /// temp

class CameraWorker {
protected:
	// BASE CLASS MEMBERS
	bool stop_ = false;
	uint64_t camera_index_;    /// temp
	uint64_t frame_index_ = 0; /// temp
	cv::VideoCapture camera_feed_;
	cv::Mat frame_;

	// THREAD CLASS MEMBERS
	std::thread th_grab_frame_;
	std::thread th_process_frame_;
	std::thread th_publish_topics_;
	std::thread th_broadcast_tfs_;

	// MUTEX CLASS MEMBERS
	std::mutex mu_camera_feed_;

	// CLASS METHODS
	// grabs all the frames at max freq so that buffer isn't clogged
	void GrabFrame();
	// processes the frame from retrieval to info extraction
	virtual void ProcessFrame() = 0;
	
	virtual void BroadcastTfs() = 0;
	virtual void PublishTopics() = 0;

public:
	// constructs the camera worker from the index of the camera on the device
	// see v4l2-ctl --list-devices
	CameraWorker(uint64_t index);
	virtual ~CameraWorker();
	void Start();
};
