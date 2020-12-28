// general c++ headers
#include <iostream> // basic input output stream manipulations

// project headers
#include "panoramic_camera_worker.h"
#include "serial_port_worker.h"

int main() {
    CameraWorker *rpi_cam = new PanoramicCameraWorker(0);
    SerialPortWorker *serial_port = new SerialPortWorker();

    rpi_cam->Start();
    serial_port->Start();

    std::cin.ignore();

    delete rpi_cam;
    delete serial_port;

    return 0;
}
