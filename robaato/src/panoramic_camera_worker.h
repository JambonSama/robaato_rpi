#pragma once

#include <cmath>
#include <tuple>

#include <iostream>

#include <opencv2/opencv.hpp>

#include "camera_worker.h"

struct Pose {
    double x = 0;
    double y = 0;
    double yaw = 0;
};

struct Position {
    double x = 0;
    double y = 0;
};

class PanoramicCameraWorker : public CameraWorker {
protected:
    // BASE CLASS MEMBERS
    std::tuple<double, double, double> position_; // [x, y, theta] robot position

    // MUTEX CLASS MEMBERS
    std::mutex mu_position_; // mutex to access the robot position

    // CLASS METHODS
    // processes the frame from retrieval to robot position
    void ProcessFrame() override;
    // updates the robot position from the frame
    void UpdatePoseFromFrame();
    // triangulates position from LED angles [rad]
    Pose Triangulate(double mpr, double rpg, double gpb,
                     double bpr, double m_angle);

public:
    // constructs the camera worker from the index of the camera on the device
    // see v4l2-ctl --list-devices
    PanoramicCameraWorker(uint64_t index);
    // destructs the camera worker
    virtual ~PanoramicCameraWorker();
};
