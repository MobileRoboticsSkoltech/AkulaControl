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
//-----------------------------//
#include "dSocket/dSocket.h"
#include "SerialMonitor.h"
//-----------------------------//
enum class SmartphoneHeader {
    REQUEST_CONN,
    JOYSTICK_COORDS,
    PING,
    STATUS,
    INVALID
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
    Server(uint16_t tPort, size_t tPacketSize, uint32_t tConnTimeout);
    ~Server();
private:
    std::atomic_bool                            mTerminate                              = false;
    std::atomic_bool                            mConnected                              = false;

    std::chrono::system_clock::time_point       mSmartphoneLastPingTime                 = std::chrono::system_clock::now();
    uint32_t                                    mTimeoutMs                              = 0;

    //----------//

    dSocket*                                    mSocketUDP                              = nullptr;

    sockaddr_in                                 mSmartphoneAddr                         = {};
    socklen_t                                   mSmartphoneAddrLen                      = 0;

    std::mutex                                  mSmartphoneMutex;

    //----------//

    size_t                                      mPacketSize                             = 0;

    uint8_t*                                    mSmartphoneReadBuffer                   = nullptr;
    uint8_t*                                    mSmartphoneWriteBuffer                  = nullptr;

    //----------//

    ///---TODO: change return type in case there are server related errors needed to be processed---///
    std::future <dSocketResult>                 mSmartphoneReadThread;
    std::future <dSocketResult>                 mSmartphoneWriteThread;
    std::future <dSocketResult>                 mSmartphoneProcessThread;

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

    SerialMonitor*                              mMonitorSTM                             = nullptr;

    //----------//

    std::future <ServerResult>                  mTimerThread;
    std::future <void>                          mSerialThread;

    //----------//

    void terminate();

    //----------//

    dSocketResult smartphoneReadCallback();
    dSocketResult smartphoneWriteCallback();
    dSocketResult smartphoneProcessCallback();

    //----------//

    bool fillSmartphoneReadBuffer(const uint8_t* tBuffer);
    bool getSmartphoneReadBuffer(uint8_t* tBuffer);

    bool fillSmartphoneWriteBuffer(const uint8_t* tBuffer);
    bool getSmartphoneWriteBuffer(uint8_t* tBuffer);

    //----------//

    ServerResult timerCallback();
    void serialCallback();
};
//-----------------------------//
#endif
