//
// Created by mrob on 30.04.2021.
//
//-----------------------------//
#include "SerialConnector.h"
//-----------------------------//
/**
 * @param tSerialPath
 * @param tSpeed
 * @param tTimeout For some unknown reason tty timeout is in deciseconds (10^-2s) but tTimeout will be in millisecond
 */
SerialConnector::SerialConnector(const std::string& tSerialPath, uint16_t tSpeed, uint32_t tTimeout, size_t tPacketSize) {
    mSerialPort = open(tSerialPath.c_str(), O_RDWR);

    if (mSerialPort < 0) {
        std::cerr << "Failed to open the port: " << errno << "(" + std::string(strerror(errno)) + ")" << std::endl;
        throw std::runtime_error("SerialConnector::constructor");
    }

    //----------//

    if (tcgetattr(mSerialPort, &mTTY) != 0) {
        std::cerr << "Failed to get attributes: " << errno << "(" + std::string(strerror(errno)) + ")" << std::endl;
        throw std::runtime_error("SerialConnector::constructor");
    }

    mTTY.c_cflag &= ~PARENB;                                             // Clear parity bit, disabling parity (most common)
    mTTY.c_cflag &= ~CSTOPB;                                             // Clear stop field, only one stop bit used in communication (most common)
    mTTY.c_cflag &= ~CSIZE;                                              // Clear all bits that set the data size
    mTTY.c_cflag |= CS8;                                                 // 8 bits per byte (most common)
    mTTY.c_cflag &= ~CRTSCTS;                                            // Disable RTS/CTS hardware flow control (most common)
    mTTY.c_cflag |= CREAD | CLOCAL;                                      // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    mTTY.c_lflag &= ~ICANON;
    mTTY.c_lflag &= ~ECHO;                                               // Disable echo
    mTTY.c_lflag &= ~ECHOE;                                              // Disable erasure
    mTTY.c_lflag &= ~ECHONL;                                             // Disable new-line echo
    mTTY.c_lflag &= ~ISIG;                                               // Disable interpretation of INTR, QUIT and SUSP
    mTTY.c_iflag &= ~(IXON | IXOFF | IXANY);                             // Turn off s/w flow ctrl
    mTTY.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);    // Disable any special handling of received bytes

    mTTY.c_oflag &= ~OPOST;                                              // Prevent special interpretation of output bytes (e.g. newline chars)
    mTTY.c_oflag &= ~ONLCR;                                              // Prevent conversion of newline to carriage return/line feed

    mTTY.c_cc[VTIME] = tTimeout / 10;                                    // Wait for up to tTimeout / 10 deciseconds, returning as soon as any data is received.
    mTTY.c_cc[VMIN] = 0;

    cfsetispeed(&mTTY, tSpeed);
    cfsetospeed(&mTTY, tSpeed);

    if (tcsetattr(mSerialPort, TCSANOW, &mTTY) != 0) {
        std::cerr << "Failed to set attributes: " << errno << "(" + std::string(strerror(errno)) + ")" << std::endl;
        throw std::runtime_error("SerialConnector::constructor");
    }

    //----------//

    mPacketSize     = tPacketSize;
}
SerialConnector::~SerialConnector() {
    close(mSerialPort);
}
//-----------------------------//
ssize_t SerialConnector::readSerial(uint8_t* tBuffer) const {
    return read(mSerialPort, tBuffer, mPacketSize);
}
ssize_t SerialConnector::writeSerial(const uint8_t* tBuffer) const {
    return write(mSerialPort, tBuffer, mPacketSize);
}