#pragma once

#include <cstdint>
#include <mutex>
#include <thread>

// project headers
#include "message.h"

class SerialPortWorker {
protected:
    // BASE CLASS MEMBERS
    bool stop_ = false;
    SensorMessage sensor_message_;
    ControlMessage control_message_;

    // THREAD CLASS MEMBERS
    std::thread th_read_write_serial_port_;

    // CLASS METHODS
    void ReadWriteSerialPort();

public:
    SerialPortWorker();
    ~SerialPortWorker();
    void Start();
};
