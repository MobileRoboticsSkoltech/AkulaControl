//
// Created by devilox on 25.03.2021.
//
//-----------------------------//
#ifndef DESKTOPSERVER_SERVER_H
#define DESKTOPSERVER_SERVER_H
//-----------------------------//
#include <thread>
#include <future>
#include <condition_variable>
#include <cstring>
#include <queue>
#include <algorithm>
//-----------------------------//
#include <yaml-cpp/yaml.h>
//-----------------------------//
#include "dSocket/dSocket.h"
#include "SerialMonitor.h"
#include "MotorPWM.h"
//-----------------------------//
enum class SmartphoneHeader {
    REQUEST_CONN,
    JOYSTICK_COORDS,
    PING,
    STATUS,
    INVALID,
    DISCONNECTED,
    LATENCY,
    ENCODER
};
enum class ServerResult {
    SUCCESS,
    SOCKET_ERROR
};
//-----------------------------//
/**
 * @description
 * Server class provides some functions for communication between desktop <-> android smartphone (UDP protocol)
 * and desktop <-> stm32 (serial packet-based connection). Almost all the operations are executed in separate
 * threads: timer, reading, packet processing, writing. In order to save resources condition variables are used to
 * suspend threads (maybe in future it will be better to migrate to C++20 and start using coroutines as long as
 * they are more efficient in terms of resources usage). For packing data in packets binary format is used with
 * custom protocol.
 */
class Server {
public:
    Server();
    ~Server();
private:
    std::atomic_bool                            mTerminate                              = false;
    std::atomic_bool                            mConnected                              = false;
    std::atomic_bool                            mSerialActive                           = false;
    std::atomic_bool                            mRecording                              = false;

    std::chrono::system_clock::time_point       mSmartphoneLastPingTime                 = std::chrono::system_clock::now();
    uint32_t                                    mTimeoutMs                              = 0;
    uint32_t                                    mTimerSleepIntervalMs                   = 0;

    //----------//

    uint32_t                                    mRecordCheckTimeoutMs                   = 0;
    std::chrono::system_clock::time_point       mRecordLastCheckTime                    = std::chrono::system_clock::now();
    std::string                                 mRecordStatusCmd;
    std::string                                 mRecordStartCmd;
    std::string                                 mRecordStopCmd;

    //----------//

    dSocket*                                    mSocketUDP                              = nullptr;
    dSocket*                                    mSensorSocketUDP                        = nullptr;

    sockaddr_in                                 mSmartphoneAddr                         = {};
    socklen_t                                   mSmartphoneAddrLen                      = 0;

    std::mutex                                  mSmartphoneMutex;

    //----------//

    size_t                                      mPacketSize                             = 0;

    uint8_t*                                    mSmartphoneReadBuffer                   = nullptr;
    uint8_t*                                    mSmartphoneWriteBuffer                  = nullptr;

    uint8_t*                                    mSensorWriteBuffer                      = nullptr;

    //----------//

    std::future <dSocketResult>                 mSmartphoneReadThread;
    std::future <dSocketResult>                 mSmartphoneWriteThread;
    std::future <dSocketResult>                 mSmartphoneProcessThread;

    std::future <dSocketResult>                 mSensorWriteThread;

    std::condition_variable                     mSmartphoneReadCV;
    std::condition_variable                     mSmartphoneWriteCV;
    std::condition_variable                     mSmartphoneProcessCV;

    std::mutex                                  mSmartphoneReadMutex;
    std::mutex                                  mSmartphoneWriteMutex;
    std::mutex                                  mSmartphoneProcessMutex;

    bool                                        mSmartphoneWriteState                   = false;
    bool                                        mSmartphoneProcessState                 = false;

    //----------//

    std::condition_variable                     mSmartphoneReadBufferCV;
    std::condition_variable                     mSmartphoneWriteBufferCV;

    std::mutex                                  mSmartphoneReadBufferMutex;
    std::mutex                                  mSmartphoneWriteBufferMutex;

    bool                                        mSmartphoneReadBufferReady              = false;
    bool                                        mSmartphoneWriteBufferReady             = false;

    //----------//

    std::condition_variable                     mSensorWriteCV;
    std::mutex                                  mSensorWriteMutex;
    bool                                        mSensorWriteState                       = false;

    std::condition_variable                     mSensorWriteBufferCV;
    std::mutex                                  mSensorWriteBufferMutex;
    bool                                        mSensorWriteBufferReady                 = false;

    //----------//

    SerialMonitor*                              mMonitorSTM                             = nullptr;
    SerialMessenger*                            mMessengerSTM                           = nullptr;
    size_t                                      mSerialPacketSize                       = 0;
    uint32_t                                    mSerialTimeout                          = 0;
    uint32_t                                    mSerialTimerSleepIntervalMs             = 0;
    uint32_t                                    mSerialPingIntervalMs                   = 0;

    //----------//

    std::string                                 mSerialPath;
    std::future <ServerResult>                  mTimerThread;
    std::future <void>                          mSerialThread;
    std::future <void>                          mSerialDataThread;

    //----------//

    MotorPWM*                                   mMotorPWM                               = nullptr;

    //----------//

    void terminate();

    //----------//

    dSocketResult smartphoneReadCallback();
    dSocketResult smartphoneWriteCallback();
    dSocketResult smartphoneProcessCallback();

    dSocketResult sensorWriteCallback();

    //----------//

    bool fillSmartphoneReadBuffer(const uint8_t* tBuffer);
    bool getSmartphoneReadBuffer(uint8_t* tBuffer);

    bool fillSmartphoneWriteBuffer(const uint8_t* tBuffer);
    bool getSmartphoneWriteBuffer(uint8_t* tBuffer);

    bool fillSensorWriteBuffer(const uint8_t* tBuffer);
    bool getSensorWriteBuffer(uint8_t* tBuffer);

    //----------//

    ServerResult timerCallback();
    void serialCallback();
    void serialDataCallback();
};
//-----------------------------//
#endif
