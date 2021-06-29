//
// Created by devilox on 25.05.2021.
//
//-----------------------------//
#include "SerialMonitor.h"
//-----------------------------//
SerialMonitor::SerialMonitor(const std::string& tSerialPath, size_t tPacketSize, SerialMessenger* tMessenger) {
    try {
        mConnector = new SerialConnector(tSerialPath, B115200, 2000, tPacketSize);
    } catch (const std::runtime_error& tExcept) {
        throw;
    }

    mPacketSize     = tPacketSize;
    mMessenger      = tMessenger;

    mReadBuffer     = new uint8_t[tPacketSize];
    mWriteBuffer    = new uint8_t[tPacketSize];
}
SerialMonitor::~SerialMonitor() {
    delete(mConnector);

    delete[](mReadBuffer);
    delete[](mWriteBuffer);
}
//-----------------------------//
///---TODO: add timeout to re-enable connection procedure and send timeout state to the smartphone---///
/**
 * @description
 * The function is the main loop for serial communication, there connection to the serial port is established and
 * packets from the port are processed
 */
void SerialMonitor::startSerialLoop() {
    auto ReadTag = PacketType::INVALID;
    auto WriteTag = PacketType::INVALID;

    //----------//

    mRunning.store(true);

    mTimerThread = std::async(std::launch::async, &SerialMonitor::timerCallback, this);

    while (mRunning.load()) {
        if (!mConnected.load()) {
            if (mConnector -> readSerial(mReadBuffer) > 0) {
                memcpy(&ReadTag, mReadBuffer, 4);

                if (ReadTag == PacketType::REQUEST_CONN) {
                    std::cout << "Request" << std::endl;
                    WriteTag = PacketType::PING;
                    memcpy(mWriteBuffer, &WriteTag, 4);
                    mConnector -> writeSerial(mWriteBuffer);
                    mConnected.store(true);

                    WriteTag = PacketType::INVALID;
                }

                ReadTag = PacketType::INVALID;
            } else {
                //---Maybe we don't need timeout here---//
                std::cerr << "Timeout" << std::endl;
            }
        } else {
            if (mConnector -> readSerial(mReadBuffer) > 0) {
                memcpy(&ReadTag, mReadBuffer, 4);

                switch (ReadTag) {
                    case PacketType::PING:
                        std::cout << "Response!" << std::endl;
                        break;
                    case PacketType::REQUEST_CONN:
                        std::cout << "Request: skip" << std::endl;
                        break;
                    case PacketType::JOYSTICK_COORDS:
                        std::cout << "Coord response!" << std::endl;
                        break;
                    case PacketType::LATENCY:
                        {
                            std::scoped_lock<std::mutex> Lock(mMessenger -> mMutex);
                            memcpy(mMessenger -> mBuffer, mReadBuffer, mPacketSize);
                            mMessenger -> mNewData.store(true);
                        }

                        mMessenger -> mDataCV.notify_one();

                        break;
                    default:
                        std::cerr << "Something wend wrong: " << static_cast <uint32_t>(ReadTag) << std::endl;
                        mRunning.store(false);
                }

                ReadTag = PacketType::INVALID;
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