//
// Created by devilox on 25.03.2021.
//
//-----------------------------//
#include "Server.h"
//-----------------------------//
Server::Server(uint16_t tPort, size_t tPacketSize) : mPort(tPort), mPacketSize(tPacketSize) {
    mSocketUDP                  = new dSocket(true);
    mSocketUDP -> init(dSocketProtocol::UDP);
    mSocketUDP -> setTimeoutOption(1000);
    mSocketUDP -> finalize(dSocketType::SERVER, tPort);

    mSmartphoneReadBuffer       = new uint8_t(tPacketSize);
    mSmartphoneWriteBuffer      = new uint8_t(tPacketSize);

    //----------//

    mSmartphoneReadThread       = std::async(std::launch::async, &Server::smartphoneReadCallback, this);
    mSmartphoneWriteThread      = std::async(std::launch::async, &Server::smartphoneWriteCallback, this);
    mSmartphoneProcessThread    = std::async(std::launch::async, &Server::smartphoneProcessCallback, this);

    mTimerThread                = std::async(std::launch::async, &Server::timerCallback, this);
}
Server::~Server() {
    delete[](mSmartphoneReadBuffer);
    delete[](mSmartphoneWriteBuffer);

    delete(mSocketUDP);
}
//-----------------------------//
dSocketResult Server::smartphoneReadCallback() {
    uint8_t Packet[mPacketSize];
    std::unique_lock <std::mutex> Lock(mSmartphoneReadMutex);
    sockaddr_in ClientAddr {};
    socklen_t ClientAddrLen;
    ssize_t ReadBytes;
    size_t ReadTotal = 0;
    dSocketResult Result;

    while (!mTerminate.load()) {
        while (ReadTotal < mPacketSize) {
            Result = mSocketUDP -> readFromAddress(Packet + ReadTotal, mPacketSize - ReadTotal, &ReadBytes,
                                                   reinterpret_cast <sockaddr*>(&ClientAddr), &ClientAddrLen);

            switch (Result) {
                case dSocketResult::SUCCESS:
                    ReadTotal += ReadBytes;
                    std::cout << ReadTotal << std::endl;
                    break;
                case dSocketResult::RECV_TIMEOUT:
                    ReadTotal = 0;
                    break;
                default:
                    ///---TODO: Add proper termination handling---///
                    return dSocketResult::READ_ERROR;
            }
        }

        ReadTotal = 0;

        mSmartphoneReadCV.wait(Lock, [this] {
            return mSmartphoneReadState || mTerminate.load();
        });

        if (mTerminate.load()) {
            ///---TODO: Add proper termination handling---///
        }

        if (!mConnected.load()) {
            std::scoped_lock <std::mutex> SmartphoneLock(mSmartphoneMutex);
            mSmartphoneAddr = ClientAddr;
            mSmartphoneAddrLen = ClientAddrLen;
        }

        fillSmartphoneReadBuffer(Packet);
        std::cout << "Here" << std::endl;
    }

    return dSocketResult::SUCCESS;
}
dSocketResult Server::smartphoneWriteCallback() {
    uint8_t Packet[mPacketSize];
    std::unique_lock <std::mutex> Lock(mSmartphoneWriteMutex);
    ssize_t WrittenBytes;
    size_t WrittenTotal = 0;
    dSocketResult Result;

    while (!mTerminate.load()) {
        mSmartphoneWriteCV.wait(Lock, [this] {
            return mSmartphoneWriteState || mTerminate.load();
        });

        if (mTerminate.load()) {
            ///---TODO: Add proper termination handling---///
        }

        Lock.unlock();

        getSmartphoneWriteBuffer(Packet);

        if (mConnected.load()) {
            while (WrittenTotal < WrittenBytes) {
                Result = mSocketUDP -> writeToAddress(Packet + WrittenTotal, mPacketSize - WrittenTotal, &WrittenBytes,
                                                      reinterpret_cast <const sockaddr*>(&mSmartphoneAddr),
                                                      mSmartphoneAddrLen);

                switch (Result) {
                    case dSocketResult::SUCCESS:
                        WrittenTotal += WrittenBytes;
                        break;
                    default:
                        ///---TODO: Add proper termination handling---///
                        return dSocketResult::READ_ERROR;
                }
            }

            WrittenTotal = 0;
        }
    }

    return dSocketResult::SUCCESS;
}
dSocketResult Server::smartphoneProcessCallback() {
    uint8_t Packet[mPacketSize];
    std::unique_lock <std::mutex> Lock(mSmartphoneProcessMutex);
    SmartphoneHeader Tag;

    while (!mTerminate.load()) {
        mSmartphoneProcessCV.wait(Lock, [this] {
            return mSmartphoneProcessState || mTerminate.load();
        });

        if (mTerminate.load()) {
            ///---TODO: Add proper termination handling---///
        }

        Lock.unlock();

        getSmartphoneReadBuffer(Packet);

        //----------//

        memcpy(&Tag, Packet, 4);

        if (Tag != SmartphoneHeader::REQUEST_CONN && !mConnected.load()) {
            std::scoped_lock <std::mutex> SmartphoneLock(mSmartphoneMutex);
            mSmartphoneAddr     = {};
            mSmartphoneAddrLen  = 0;

            continue;
        }

        switch (Tag) {
            case SmartphoneHeader::REQUEST_CONN:
                mConnected.store(true);
                break;
            case SmartphoneHeader::JOYSTICK_COORDS:
                ///---TODO: Add coords handling---///

                break;
            case SmartphoneHeader::PING:
                ///---TODO: Add ping handling---///

                break;
            case SmartphoneHeader::STATUS:
                ///---TODO: Add status handling---///

                break;
        }
    }

    return dSocketResult::SUCCESS;
}
//-----------------------------//
bool Server::fillSmartphoneReadBuffer(const uint8_t* tBuffer) {
    std::unique_lock <std::mutex> Lock(mSmartphoneReadBufferMutex);

    mSmartphoneReadBufferCV.wait(Lock, [this] {
        return !mSmartphoneReadBufferReady || mTerminate.load();
    });

    if (mTerminate.load()) {
        return false;
    }

    memcpy(mSmartphoneReadBuffer, tBuffer, mPacketSize);

    {
        std::scoped_lock <std::mutex> StateLock(mSmartphoneReadMutex);          //---Lost wake up prevention---//
        mSmartphoneReadState = false;
    }

    {
        std::scoped_lock <std::mutex> StateLock(mSmartphoneProcessMutex);       //---Lost wake up prevention---//
        mSmartphoneProcessState = true;
    }

    mSmartphoneReadBufferReady = true;

    mSmartphoneProcessCV.notify_one();
    mSmartphoneReadBufferCV.notify_one();

    return true;
}
bool Server::getSmartphoneReadBuffer(uint8_t* tBuffer) {
    std::unique_lock <std::mutex> Lock(mSmartphoneReadBufferMutex);

    mSmartphoneReadBufferCV.wait(Lock, [this] {
        return mSmartphoneReadBufferReady || mTerminate.load();
    });

    if (mTerminate.load()) {
        return false;
    }

    memcpy(tBuffer, mSmartphoneReadBuffer, mPacketSize);

    {
        std::scoped_lock <std::mutex> StateLock(mSmartphoneReadMutex);          //---Lost wake up prevention---//
        mSmartphoneReadState = true;
    }

    {
        std::scoped_lock <std::mutex> StateLock(mSmartphoneProcessMutex);       //---Lost wake up prevention---//
        mSmartphoneProcessState = false;
    }

    mSmartphoneReadCV.notify_one();
    mSmartphoneReadBufferCV.notify_one();

    return true;
}

bool Server::fillSmartphoneWriteBuffer(const uint8_t* tBuffer) {
    std::unique_lock <std::mutex> Lock(mSmartphoneWriteBufferMutex);

    mSmartphoneWriteBufferCV.wait(Lock, [this] {
        return !mSmartphoneWriteBufferReady || mTerminate.load();
    });

    if (mTerminate.load()) {
        return false;
    }

    memcpy(mSmartphoneWriteBuffer, tBuffer, mPacketSize);

    {
        std::scoped_lock <std::mutex> StateLock(mSmartphoneWriteMutex);         //---Lost wake up prevention---//
        mSmartphoneWriteState = true;
    }

    {
        std::scoped_lock <std::mutex> StateLock(mSmartphoneProcessMutex);       //---Lost wake up prevention---//
        mSmartphoneProcessState = false;
    }

    mSmartphoneWriteBufferReady = true;

    mSmartphoneWriteCV.notify_one();
    mSmartphoneWriteBufferCV.notify_one();

    return true;
}
bool Server::getSmartphoneWriteBuffer(uint8_t* tBuffer) {
    std::unique_lock <std::mutex> Lock(mSmartphoneWriteBufferMutex);

    mSmartphoneWriteBufferCV.wait(Lock, [this] {
        return mSmartphoneWriteBufferReady || mTerminate.load();
    });

    if (mTerminate.load()) {
        return false;
    }

    memcpy(tBuffer, mSmartphoneWriteBuffer, mPacketSize);

    {
        std::scoped_lock <std::mutex> StateLock(mSmartphoneWriteMutex);         //---Lost wake up prevention---//
        mSmartphoneWriteState = false;
    }

    {
        std::scoped_lock <std::mutex> StateLock(mSmartphoneProcessMutex);       //---Lost wake up prevention---//
        mSmartphoneProcessState = true;
    }

    mSmartphoneWriteBufferReady = false;

    mSmartphoneProcessCV.notify_one();
    mSmartphoneWriteBufferCV.notify_one();

    return true;
}
//-----------------------------//
void Server::timerCallback() {
    uint8_t Packet[mPacketSize];
    SmartphoneHeader Tag = SmartphoneHeader::PING;

    memcpy(Packet, &Tag, 4);

    while (!mTerminate.load()) {
        if (mConnected.load()) {
            fillSmartphoneWriteBuffer(Packet);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}