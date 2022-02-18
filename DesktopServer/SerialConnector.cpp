//
// Created by devilox on 30.04.2021.
//
//-----------------------------//
#include "SerialConnector.h"
//-----------------------------//
/**
 * @descption
 * Constructor opens a serial port and sets all the necessary bits (maybe not all of them)
 * @param tSerialPath <b>/dev/tty*</b> or any symbolic link
 * @param tSpeed Baud rate defined in bits/termios.h
 * @param tTimeout Select timeout in milliseconds
 * @param tPacketSize Number of bytes per packet
 */
SerialConnector::SerialConnector(const std::string& tSerialPath, uint16_t tSpeed, uint32_t tTimeout, size_t tPacketSize) {
    mSerialPort     = open(tSerialPath.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    mTimeoutMs      = tTimeout;
    mPacketSize     = tPacketSize;

    if (mSerialPort < 0) {
        std::cerr << "Failed to open the port: " << errno << "(" + std::string(strerror(errno)) + ")" << std::endl;
        throw std::runtime_error("SerialConnector::constructor");
    }

    //----------//

    if (tcgetattr(mSerialPort, &mTTY) != 0) {
        close(mSerialPort);
        std::cerr << "Failed to get attributes: " << errno << "(" + std::string(strerror(errno)) + ")" << std::endl;
        throw std::runtime_error("SerialConnector::constructor");
    }

    mTTY.c_cflag &= ~PARENB;                                                // Clear parity bit, disabling parity (most common)
    mTTY.c_cflag &= ~CSTOPB;                                                // Clear stop field, only one stop bit used in communication (most common)
    mTTY.c_cflag &= ~CSIZE;                                                 // Clear all bits that set the data size
    mTTY.c_cflag |= CS8;                                                    // 8 bits per byte (most common)
    mTTY.c_cflag &= ~CRTSCTS;                                               // Disable RTS/CTS hardware flow control (most common)
    mTTY.c_cflag |= CREAD | CLOCAL;                                         // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    mTTY.c_lflag &= ~ICANON;
    mTTY.c_lflag &= ~ECHO;                                                  // Disable echo
    mTTY.c_lflag &= ~ECHOE;                                                 // Disable erasure
    mTTY.c_lflag &= ~ECHONL;                                                // Disable new-line echo
    mTTY.c_lflag &= ~ISIG;                                                  // Disable interpretation of INTR, QUIT and SUSP
    mTTY.c_iflag &= ~(IXON | IXOFF | IXANY);                                // Turn off s/w flow ctrl
    mTTY.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);       // Disable any special handling of received bytes

    mTTY.c_oflag &= ~OPOST;                                                 // Prevent special interpretation of output bytes (e.g. newline chars)
    mTTY.c_oflag &= ~ONLCR;                                                 // Prevent conversion of newline to carriage return/line feed

    mTTY.c_cc[VTIME] = 0;                                                   // Polling time interval
    mTTY.c_cc[VMIN] = 0;

    cfsetispeed(&mTTY, tSpeed);
    cfsetospeed(&mTTY, tSpeed);

    if (tcsetattr(mSerialPort, TCSANOW, &mTTY) != 0) {
        close(mSerialPort);
        std::cerr << "Failed to set attributes: " << errno << "(" + std::string(strerror(errno)) + ")" << std::endl;
        throw std::runtime_error("SerialConnector::constructor");
    }
}
SerialConnector::~SerialConnector() {
    close(mSerialPort);
}
//-----------------------------//
/**
 * @description
 * Checks how many bytes are available in the read buffer <b>(not tBuffer)</b> and fills <b>tBuffer</b>
 * @param tBuffer mBuffer to put new bytes to
 * @return Returns number of bytes read if success, 0 when disconnected or timeout and -1 otherwise
 */
ssize_t SerialConnector::readSerial(uint8_t* tBuffer) const {
    ssize_t BytesTotalRead = 0;
    ssize_t BytesRead;
    int Available;

    while (BytesTotalRead < mPacketSize) {
        if (checkNewData()) {
            if (ioctl(mSerialPort, TIOCINQ, &Available) != -1) {
                BytesRead = read(mSerialPort, tBuffer + BytesTotalRead, mPacketSize - BytesTotalRead);

                if (BytesRead == 0) {
                    return 0;
                } else if (BytesRead < 0) {
                    std::cerr << "readSerial: read error" << std::endl;
                    return -1;
                } else {
                    BytesTotalRead += BytesRead;
                }
            } else {
                std::cerr << "readSerial: ioctl error" << std::endl;
                return -1;
            }
        } else {
            return 0;
        }
    }

    return BytesTotalRead;
}
/**
 * @description
 * Writes to serial port from the provided buffer <b>tBuffer</b>
 * @param tBuffer mBuffer to write bytes from
 * @return Returns the number of bytes written or negative value (error) otherwise
 */
ssize_t SerialConnector::writeSerial(const uint8_t* tBuffer) const {
    return write(mSerialPort, tBuffer, mPacketSize);
}
//-----------------------------//
/**
 * @description
 * Checks whether there are bytes available in the read buffer and waits for some specified time before
 * printing timeout
 * @return Returns 0 in case of timeout, -1 in case of error, 1 otherwise
 */
int8_t SerialConnector::checkNewData() const {
    fd_set ReadFDs;
    int Count;
    timespec Timeout {
            .tv_sec     = mTimeoutMs / 1000,
            .tv_nsec    = mTimeoutMs % 1000 * 1'000'000
    };

    FD_ZERO(&ReadFDs);
    FD_SET(mSerialPort, &ReadFDs);

    Count = pselect(mSerialPort + 1, &ReadFDs, nullptr, nullptr, &Timeout, nullptr);

    if (Count > 0) {
        return 1;
    } else {
        if (Count == 0) {
            std::cerr << "checkNewData: timeout" << std::endl;
            return 0;
        } else {
            std::cerr << "checkNewData: pselect error " << Count << std::endl;
            return -1;
        }
    }
}