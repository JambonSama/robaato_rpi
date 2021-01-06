#include "camera_worker.h"

using namespace std::chrono_literals;

CameraWorker::CameraWorker(uint64_t index)
	: camera_index_(index), /// temp
	  camera_feed_(index)   //
{
	std::cout << "cw constructor" << std::endl; /// temp

	/// exceptions
	if (!camera_feed_.isOpened()) {
		std::cout << "Error opening video stream or file" << std::endl;
		return;
	}
}

CameraWorker::~CameraWorker() {
	std::cout << "cw destructor" << std::endl; /// temp

	stop_ = true;

	if (th_grab_frame_.joinable()) {
		th_grab_frame_.join();
	}
	std::cout << "cw destructor grab frame" << std::endl; /// temp

	if (th_process_frame_.joinable()) {
		th_process_frame_.join();
	}
	std::cout << "cw destructor process frame" << std::endl; /// temp

	if (th_publish_topics_.joinable()) {
		th_publish_topics_.join();
	}

	if (th_broadcast_tfs_.joinable()) {
		th_broadcast_tfs_.join();
	}
}

void CameraWorker::GrabFrame() {
	std::this_thread::sleep_for(1ms);               // don't anger higher powers
	std::cout << "cw grab frame init" << std::endl; /// temp

	while (!stop_) {
		mu_camera_feed_.lock();
		camera_feed_.grab();
		mu_camera_feed_.unlock();
		// RPi cam frame rate is approx. 33 Hz => T ~ 33 ms
		// Frontal camera frame rate is approx.
		std::this_thread::sleep_for(3ms);
	}
}

void CameraWorker::Start() {
	th_grab_frame_ = std::thread(&CameraWorker::GrabFrame, this);
	th_process_frame_ = std::thread(&CameraWorker::ProcessFrame, this);
	th_publish_topics_ = std::thread(&CameraWorker::PublishTopics, this);
	th_broadcast_tfs_ = std::thread(&CameraWorker::BroadcastTfs, this);
}
