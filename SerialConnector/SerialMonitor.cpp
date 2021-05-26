//
// Created by devilox on 25.05.2021.
//
//-----------------------------//
#include "SerialMonitor.h"
//-----------------------------//
SerialMonitor::SerialMonitor(const std::string& tSerialPath, size_t tPacketSize) {
    ///---TODO: fix timeout---///
    mConnector      = new SerialConnector(tSerialPath, B115200, 1000, tPacketSize);

    mReadBuffer     = new uint8_t[tPacketSize];
    mWriteBuffer    = new uint8_t[tPacketSize];
}
SerialMonitor::~SerialMonitor() {
    delete(mConnector);

    delete[](mReadBuffer);
    delete[](mWriteBuffer);
}
//-----------------------------//
void SerialMonitor::startSerialLoop() {
    auto ReadTag = PacketType::INVALID;
    auto WriteTag = PacketType::INVALID;

    //----------//

    mRunning.store(true);

    ///---TODO: change void to smth and deal with this value---///
    mTimerThread = std::async(std::launch::async, &SerialMonitor::timerCallback, this);

    while (mConnector && mRunning.load()) {
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
                    default:
                        std::cerr << "Something wend wrong: " << static_cast <uint32_t>(ReadTag) << std::endl;
                        mRunning.store(false);
                }

                ReadTag = PacketType::INVALID;
            }
        }
    }
}
//-----------------------------//
void SerialMonitor::timerCallback() {
    while (mRunning.load()) {
        mPingDuration = std::chrono::system_clock::now() - mLastPing;

        if (mConnector && mConnected.load() && (mPingDuration.count() * 1000 > mPingIntervalMs)) {
            auto Tag = static_cast <uint32_t>(PacketType::PING);
            memcpy(mWriteBuffer, &Tag, 4);
            mConnector->writeSerial(mWriteBuffer);

            mLastPing = std::chrono::system_clock::now();

            std::cout << "Ping" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}