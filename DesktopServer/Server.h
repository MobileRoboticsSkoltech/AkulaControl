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
//-----------------------------//
class Server {
public:
    Server(uint16_t tPort, size_t tPacketSize, uint32_t tConnTimeout);
    ~Server();

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

    void timerCallback();
    void serialCallback();
private:
    std::atomic_bool                            mTerminate                              = false;
    std::atomic_bool                            mConnected                              = false;

    std::chrono::system_clock::time_point       mSmartphoneLastPacketTime               = std::chrono::system_clock::now();
    uint32_t                                    mTimeoutMs                              = 0;

    //----------//

    dSocket*                                    mSocketUDP                              = nullptr;
    uint16_t                                    mPort                                   = 0;

    sockaddr_in                                 mSmartphoneAddr                         = {};
    socklen_t                                   mSmartphoneAddrLen                      = 0;

    std::mutex                                  mSmartphoneMutex;

    //----------//

    size_t                                      mPacketSize                             = 0;

    uint8_t*                                    mSmartphoneReadBuffer                   = nullptr;
    uint8_t*                                    mSmartphoneWriteBuffer                  = nullptr;

    std::queue <uint8_t*>                       mReadQueue;
    std::queue <uint8_t*>                       mWriteQueue;

    //----------//

    std::future <dSocketResult>                 mSmartphoneReadThread;
    std::future <dSocketResult>                 mSmartphoneWriteThread;
    std::future <dSocketResult>                 mSmartphoneProcessThread;

    std::condition_variable                     mSmartphoneReadCV;
    std::condition_variable                     mSmartphoneWriteCV;
    std::condition_variable                     mSmartphoneProcessCV;

    std::mutex                                  mSmartphoneReadMutex;
    std::mutex                                  mSmartphoneWriteMutex;
    std::mutex                                  mSmartphoneProcessMutex;

    bool                                        mSmartphoneReadState                    = true;
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

    std::future <void>                          mTimerThread;
    std::future <void>                          mSerialThread;
};
//-----------------------------//
#endif
