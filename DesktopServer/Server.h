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
//-----------------------------//
#include "dSocket/dSocket.h"
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
    Server(uint16_t tPort, size_t tPacketSize);
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
private:
    std::atomic_bool                mTerminate                              = false;
    std::atomic_bool                mConnected                              = false;

    //----------//

    dSocket*                        mSocketUDP                              = nullptr;
    uint16_t                        mPort                                   = 0;

    sockaddr_in                     mSmartphoneAddr                         = {};
    socklen_t                       mSmartphoneAddrLen                      = 0;

    std::mutex                      mSmartphoneMutex;

    //----------//

    size_t                          mPacketSize                             = 0;

    uint8_t*                        mSmartphoneReadBuffer                   = nullptr;
    uint8_t*                        mSmartphoneWriteBuffer                  = nullptr;

    //----------//

    std::future <dSocketResult>     mSmartphoneReadThread;
    std::future <dSocketResult>     mSmartphoneWriteThread;
    std::future <dSocketResult>     mSmartphoneProcessThread;

    std::condition_variable         mSmartphoneReadCV;
    std::condition_variable         mSmartphoneWriteCV;
    std::condition_variable         mSmartphoneProcessCV;

    std::mutex                      mSmartphoneReadMutex;
    std::mutex                      mSmartphoneWriteMutex;
    std::mutex                      mSmartphoneProcessMutex;

    bool                            mSmartphoneReadState                    = true;
    bool                            mSmartphoneWriteState                   = false;
    bool                            mSmartphoneProcessState                 = false;

    //----------//

    std::condition_variable         mSmartphoneReadBufferCV;
    std::condition_variable         mSmartphoneWriteBufferCV;

    std::mutex                      mSmartphoneReadBufferMutex;
    std::mutex                      mSmartphoneWriteBufferMutex;

    bool                            mSmartphoneReadBufferReady              = false;
    bool                            mSmartphoneWriteBufferReady             = false;

    //----------//

    std::future <void>              mTimerThread;
};
//-----------------------------//
#endif
