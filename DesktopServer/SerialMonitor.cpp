//
// Created by devilox on 25.05.2021.
//
//-----------------------------//
#include "SerialMonitor.h"
//-----------------------------//
SerialMonitor::SerialMonitor(const std::string& tSerialPath,
                             size_t tPacketSize,
                             SerialMessenger* tMessenger,
                             uint32_t tSerialTimeoutMs,
                             uint32_t tTimerSleepIntervalMs,
                             uint32_t tPingIntervalMs) {
    mPacketSize             = tPacketSize;
    mSerialPath             = tSerialPath;
    mMessenger              = tMessenger;
    mSerialTimeout          = tSerialTimeoutMs;
    mTimerSleepIntervalMs   = tTimerSleepIntervalMs;
    mPingIntervalMs         = tPingIntervalMs;

    mReadBuffer     = new uint8_t[tPacketSize];
    mWriteBuffer    = new uint8_t[tPacketSize];

    mRunning.store(true);
}
SerialMonitor::~SerialMonitor() {
    mRunning.store(false);

    if (mTimerThread.valid()) {
        mTimerThread.wait();
    }

    delete(mConnector);

    delete[](mReadBuffer);
    delete[](mWriteBuffer);
}
//-----------------------------//
/**
 * @description
 * The function is the main loop for serial communication, there connection to the serial port is established and
 * packets from the port are processed
 */
void SerialMonitor::startSerialLoop() {
    auto ReadTag = PacketType::INVALID;
    auto WriteTag = PacketType::INVALID;

    ssize_t ReadBytes;

    //----------//

    try {
        mConnector = new SerialConnector(mSerialPath, B115200, mSerialTimeout, mPacketSize);
    } catch (const std::runtime_error &tExcept) {
        mRunning.store(false);
        throw std::runtime_error("Serial connector creation failed!");
    }

    mTimerThread = std::async(std::launch::async, &SerialMonitor::timerCallback, this);

    while (mRunning.load()) {
        if (!mConnected.load()) {
            if ((ReadBytes = mConnector -> readSerial(mReadBuffer)) > 0) {
                memcpy(&ReadTag, mReadBuffer, 4);

                if (ReadTag == PacketType::REQUEST_CONN) {
                    std::cout << "Request" << std::endl;
                    WriteTag = PacketType::PING;
                    memcpy(mWriteBuffer, &WriteTag, 4);

                    if (mConnector -> writeSerial(mWriteBuffer) < 0) {
                        mRunning.store(false);
                        throw std::runtime_error("writeSerial failed!");
                    }

                    mConnected.store(true);
                    WriteTag = PacketType::INVALID;
                }

                ReadTag = PacketType::INVALID;
            } else if (ReadBytes < 0) {
                mRunning.store(false);
                throw std::runtime_error("readSerial failed!");
            }
        } else {
            if ((ReadBytes = mConnector -> readSerial(mReadBuffer)) > 0) {
                memcpy(&ReadTag, mReadBuffer, 4);

                switch (ReadTag) {
                    case PacketType::PING:
                        std::cout << "Response!" << std::endl;
                        break;
                    case PacketType::REQUEST_CONN:
                        std::cout << "Request: skip" << std::endl;
                        break;
                    case PacketType::JOYSTICK_COORDS:
                        break;
                    case PacketType::ENCODER:
                    case PacketType::LATENCY:
                        {
                            std::scoped_lock<std::mutex> Lock(mMessenger -> mMutex);
                            memcpy(mMessenger -> mBuffer, mReadBuffer, mPacketSize);
                            mMessenger -> mNewData.store(true);
                        }

                        mMessenger -> mDataCV.notify_one();
                        break;
                    case PacketType::INVALID:
                        {
                            uint32_t Tag;
                            memcpy(&Tag, mReadBuffer + 4, 4);
                            std::cerr << "Invalid " << Tag << std::endl;
                        }

                        break;
                    default:
                        mRunning.store(false);
                        throw std::runtime_error("Unknown packet: " + std::to_string(static_cast <uint32_t>(ReadTag)));
                }

                ReadTag = PacketType::INVALID;
            } else if (ReadBytes < 0) {
                mRunning.store(false);
                throw std::runtime_error("readSerial failed!");
            }
        }
    }
}
void SerialMonitor::terminate() {
    mRunning.store(false);

    if (mTimerThread.valid()) {
        mTimerThread.wait();
    }
}
//-----------------------------//
/**
 * @description
 * Temporary function (maybe idk)
 */
void SerialMonitor::sendPWM(int32_t tLeftPWM, int32_t tRightPWM) {
    auto Tag = static_cast <uint32_t>(PacketType::JOYSTICK_COORDS);

    memcpy(mWriteBuffer, &Tag, 4);
    memcpy(mWriteBuffer + 4, &tLeftPWM, 4);
    memcpy(mWriteBuffer + 8, &tRightPWM, 4);

    if (mConnector) {
        mConnector -> writeSerial(mWriteBuffer);
    }
}
void SerialMonitor::sendLatencyTest() {
    auto Tag = static_cast <uint32_t>(PacketType::LATENCY);
    memcpy(mWriteBuffer, &Tag, 4);

    if (mConnector) {
        mConnector -> writeSerial(mWriteBuffer);
    }
}
void SerialMonitor::sendStop() {
    auto Tag = static_cast <uint32_t>(PacketType::STOP);
    memcpy(mWriteBuffer, &Tag, 4);

    if (mConnector) {
        mConnector -> writeSerial(mWriteBuffer);
    }
}
//-----------------------------//
bool SerialMonitor::isConnected() {
    return mConnected.load();
}
//-----------------------------//
/**
 * @description
 * Timer function is executed in a separate thread during server's uptime. The thread wakes up periodically to
 * checks if it is necessary to send ping to the smartphone.
 */
void SerialMonitor::timerCallback() {
    while (mRunning.load()) {
        mPingDuration = std::chrono::system_clock::now() - mLastPing;

        if (mConnector && mConnected.load() && (mPingDuration.count() * 1000 > mPingIntervalMs)) {
            auto Tag = static_cast <uint32_t>(PacketType::PING);
            memcpy(mWriteBuffer, &Tag, 4);
            mConnector -> writeSerial(mWriteBuffer);

            mLastPing = std::chrono::system_clock::now();

            std::cout << "Ping" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(mTimerSleepIntervalMs));
    }
}