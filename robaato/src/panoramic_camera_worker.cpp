#include "panoramic_camera_worker.h"

namespace {
// h/s/v for hue, value, saturation
// lower case / upper case for min Max
// r/g/b/y for red, green, blue, yellow

// red
const uint16_t hr = 230 * 180 / 255;    //246
const uint16_t sr = 130;//130;     //130;    //171;
const uint16_t vr = 130;    //130;    //137;
const uint16_t HR = 255 * 180 / 255;
const uint16_t SR = 220;
const uint16_t VR = 180;

// green
const uint16_t hg = 21 * 180 / 255;
const uint16_t sg = 13; //53;
const uint16_t vg = 100;
const uint16_t HG = 130 * 180 / 255; //88
const uint16_t SG = 200; //127; 
const uint16_t VG = 255; //225;

// blue
const uint16_t hb = 145 * 180 / 255;
const uint16_t sb = 140;
const uint16_t vb = 165;
const uint16_t HB = 180 * 180 / 255;
const uint16_t SB = 230;
const uint16_t VB = 255;

// magenta
const uint16_t hm = 189 * 180 / 255;
const uint16_t sm = 80; //60; //156;
const uint16_t vm = 150;    //138;
const uint16_t HM = 213 * 180 / 255;
const uint16_t SM = 227;
const uint16_t VM = 250; //220;

const uint16_t arena_width = 800;		//[cm]
const uint16_t arena_height = 800;		//[cm]

// image metrics
const uint16_t frame_width = 640;
const uint16_t frame_height = 480;
const cv::Point2f frame_center(335, 270);
const double outer_radius = 190;

const uint16_t m_pos_x = 0;
const uint16_t m_pos_y = 0;
const uint16_t g_pos_x = 800;
const uint16_t g_pos_y = 800;
const double cam_pano_rot_init = 4*M_PI/3;
} // namespace

PanoramicCameraWorker::PanoramicCameraWorker(uint64_t index) : CameraWorker(index) {
    std::cout << "pcw constructor" << std::endl;
}

PanoramicCameraWorker::~PanoramicCameraWorker() {
    std::cout << "pcw destructor" << std::endl;
}

void PanoramicCameraWorker::ProcessFrame() {
    // don't anger higher powers
    std::this_thread::sleep_for(1ms);
    std::cout << "pcw process frame init" << std::endl; /// temp
    bool retval = true;                                 /// temp

    while (!stop_) {
        // std::cout << "pcw process frame while" << std::endl; /// temp

        // get the latest frame
        mu_camera_feed_.lock();
        camera_feed_.retrieve(frame_);
        mu_camera_feed_.unlock();
    
        // update position from frame
        this->UpdatePoseFromFrame();
    
        /// temp
        std::string filename =
            "test" + std::to_string(camera_index_) + "_" + std::to_string(frame_index_) + ".png";
        retval = cv::imwrite(filename, frame_);
        if (!retval) {
            std::cout << "ò_ó writing img\n";
            return;
        }
        ++frame_index_;
        // std::cout << "image num " << frame_index_ << std::endl;

        // sleep
        std::this_thread::sleep_for(1s);
        
    }
}

void PanoramicCameraWorker::UpdatePoseFromFrame() {
}

Pose PanoramicCameraWorker::Triangulate(double mpr, double rpg, double gpb, double bpr, double m_angle) {
/*
    return 
        robot_angle
        [x,y] position from angles MPR, RPG, GPB, BPM
    M: magneta Led
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
    return {};
}
