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

SerialPortWorker::SerialPortWorker() {}

SerialPortWorker::~SerialPortWorker() {
    std::cout << "spw destructor" << std::endl; /// temp

    stop_ = true;

    if (th_read_write_serial_port_.joinable()) {
        th_read_write_serial_port_.join();
    }
    std::cout << "spw destructor read/write serial port" << std::endl; /// temp
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

    while (!stop_) {
        // write to serial port
        control_message_.gate_position = false;
        control_message_.left_wheel_direction = true;
        control_message_.right_wheel_direction = false;
        control_message_.left_wheel_speed = 2000;
        control_message_.right_wheel_speed = 2000;
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

void SerialPortWorker::Start() {
    th_read_write_serial_port_ = std::thread(&SerialPortWorker::ReadWriteSerialPort, this);
}
