//
// Created by devilox on 25.03.2021.
//
//-----------------------------//
#include "Server.h"
//-----------------------------//
/**
 * @description
 * Constructor initializes a socket for communication with android smartphone and serial monitor,
 * starts read, write, process, timer and serial main loop thread
 */
Server::Server() {
    YAML::Node Config       = YAML::LoadFile("config/control.yaml");

    //----------//

    mSerialPath                     = Config["serial"]["port"].as <std::string>();
    mSerialPacketSize               = Config["serial"]["packet_size"].as <size_t>();
    mSerialTimeout                  = Config["serial"]["timeout_ms"].as <uint32_t>();
    mSerialTimerSleepIntervalMs     = Config["serial"]["timer_wakeup_cd_ms"].as <uint32_t>();
    mSerialPingIntervalMs           = Config["serial"]["ping_cd_ms"].as <uint32_t>();

    //----------//

    auto MaxPWM                     = Config["pwm"]["max_pwm"].as <uint32_t>();
    auto MaxCoordRadius             = Config["pwm"]["max_coord_radius"].as <float>();

    mMotorPWM                       = new MotorPWM(MaxPWM, MaxCoordRadius);

    //----------//

    mRecordCheckTimeoutMs           = Config["record_cmd"]["status_check_cd_ms"].as <uint32_t>();
    mRecordStatusCmd                = Config["record_cmd"]["status"].as <std::string>();
    mRecordStartCmd                 = Config["record_cmd"]["start"].as <std::string>();
    mRecordStopCmd                  = Config["record_cmd"]["stop"].as <std::string>();

    //----------//

    auto Port                       = Config["smartphone"]["port"].as <uint16_t>();
    mPacketSize                     = Config["smartphone"]["packet_size"].as <size_t>();
    mTimeoutMs                      = Config["smartphone"]["timeout_ms"].as <uint32_t>();
    mTimerSleepIntervalMs           = Config["smartphone"]["timer_wakeup_cd_ms"].as <uint32_t>();

    dSocketResult SocketRes;
    mSocketUDP = new dSocket(true);

    if ((SocketRes = mSocketUDP -> init(dSocketProtocol::UDP)) != dSocketResult::SUCCESS) {
        switch (SocketRes) {
            case dSocketResult::WRONG_PROTOCOL:
                throw std::runtime_error("Server::Server: WRONG_PROTOCOL");
            case dSocketResult::CREATE_FAILURE:
                throw std::runtime_error("Server::Server: CREATE_FAILURE, " + mSocketUDP -> getLastError());
            default:
                throw std::runtime_error("Server::Server: this should have not occurred!");
        }
    }

    if ((SocketRes = mSocketUDP -> finalize(dSocketType::SERVER, Port)) != dSocketResult::SUCCESS) {
        switch (SocketRes) {
            case dSocketResult::WRONG_PROTOCOL:
                throw std::runtime_error("Server::Server: WRONG_PROTOCOL");
            case dSocketResult::NO_SOCKET_TYPE:
                throw std::runtime_error("Server::Server: NO_SOCKET_TYPE");
            case dSocketResult::BIND_FAILURE:
                throw std::runtime_error("Server::Server: BIND_FAILURE, " + mSocketUDP -> getLastError());
            case dSocketResult::LISTEN_FAILURE:
                throw std::runtime_error("Server::Server: LISTEN_FAILURE, " + mSocketUDP -> getLastError());
            case dSocketResult::ADDRESS_CONVERSION_FAILURE:
                throw std::runtime_error("Server::Server: ADDRESS_CONVERSION_FAILURE, " + mSocketUDP -> getLastError());
            default:
                throw std::runtime_error("Server::Server: this should have not occurred!");
        }
    }

    //----------//

    mSmartphoneReadBuffer       = new uint8_t[mPacketSize];
    mSmartphoneWriteBuffer      = new uint8_t[mPacketSize];

    //----------//

    mSmartphoneReadThread       = std::async(std::launch::async, &Server::smartphoneReadCallback, this);
    mSmartphoneWriteThread      = std::async(std::launch::async, &Server::smartphoneWriteCallback, this);
    mSmartphoneProcessThread    = std::async(std::launch::async, &Server::smartphoneProcessCallback, this);

    mTimerThread                = std::async(std::launch::async, &Server::timerCallback, this);
    mSerialThread               = std::async(std::launch::async, &Server::serialCallback, this);
}
Server::~Server() {
    dSocketResult SocketRes;
    ServerResult ServerRes;

    terminate();

    if (mSmartphoneReadThread.valid()) {
        if ((SocketRes = mSmartphoneReadThread.get()) != dSocketResult::SUCCESS) {
            std::cerr << "mSmartphoneReadThread error: " << static_cast <uint32_t>(SocketRes) << std::endl;
        }
    }

    if (mSmartphoneWriteThread.valid()) {
        if ((SocketRes = mSmartphoneWriteThread.get()) != dSocketResult::SUCCESS) {
            std::cerr << "mSmartphoneReadThread error: " << static_cast <uint32_t>(SocketRes) << std::endl;
        }
    }

    if (mSmartphoneProcessThread.valid()) {
        if ((SocketRes = mSmartphoneProcessThread.get()) != dSocketResult::SUCCESS) {
            std::cerr << "mSmartphoneReadThread error: " << static_cast <uint32_t>(SocketRes) << std::endl;
        }
    }

    if (mTimerThread.valid()) {
        if ((ServerRes = mTimerThread.get()) != ServerResult::SUCCESS) {
            std::cerr << "mTimerThread error: " << static_cast <uint32_t>(ServerRes) << std::endl;
        }
    }

    if (mSerialThread.valid()) {
        mSerialThread.wait();
    }

    delete[](mSmartphoneReadBuffer);
    delete[](mSmartphoneWriteBuffer);
}
//-----------------------------//
/**
 * @description
 * Function is used to properly terminate all the threads in case there is a critical error. Socket is deleted here
 * because it reads data in blocking mode, so, deleting it will force the read() function to return 0 and lead to
 * the thread finishing execution
 */
void Server::terminate() {
    mTerminate.store(true);

    mSmartphoneReadBufferCV.notify_all();
    mSmartphoneWriteBufferCV.notify_all();

    delete(mSocketUDP);

    mSmartphoneReadCV.notify_one();
    mSmartphoneWriteCV.notify_one();
    mSmartphoneProcessCV.notify_one();

    mMonitorSTM -> terminate();
}
//-----------------------------//
/**
 * @description
 * Function is used for obtaining data from smartphone socked and called in a separate thread. Condition variable is
 * not used as long as socket operates in the blocking state, in case of disconnection read function must return 0
 * and the loop continues. Decryption can also be added to this function before moving data buffer to the processing
 * stage
 * @return Returns dSocket state for proper termination in case of connection failure
 */
dSocketResult Server::smartphoneReadCallback() {
    uint8_t Packet[mPacketSize];
    std::unique_lock <std::mutex> Lock(mSmartphoneReadMutex);
    sockaddr_in ClientAddr {};
    socklen_t ClientAddrLen = sizeof(ClientAddr);
    ssize_t ReadBytes;
    size_t ReadTotal = 0;
    dSocketResult Result;

    while (!mTerminate.load()) {
        while (ReadTotal < mPacketSize && !mTerminate.load()) {
            Result = mSocketUDP -> readUDP(Packet + ReadTotal, mPacketSize - ReadTotal, &ReadBytes,
                                           reinterpret_cast <sockaddr*>(&ClientAddr), &ClientAddrLen);

            switch (Result) {
                case dSocketResult::SUCCESS:
                    ReadTotal += ReadBytes;
                    break;
                case dSocketResult::RECV_TIMEOUT:
                    ReadTotal = 0;
                    break;
                default:
                    ///---TODO: skip some particular errors and terminate on others---///
                    terminate();
                    return dSocketResult::READ_ERROR;
            }
        }

        ReadTotal = 0;

        if (mTerminate.load()) {
            break;
        }

        if (!mConnected.load()) {
            std::scoped_lock <std::mutex> SmartphoneLock(mSmartphoneMutex);
            mSmartphoneAddr = ClientAddr;
            mSmartphoneAddrLen = ClientAddrLen;
        } else {
            if (ClientAddr.sin_addr.s_addr != mSmartphoneAddr.sin_addr.s_addr) {
                continue;
            }
        }

        if (!fillSmartphoneReadBuffer(Packet)) {
            break;
        }
    }

    return dSocketResult::SUCCESS;
}
/**
 * @description
 * Function is used for sending data to a smartphone. Most of the time in sleeps on the conditional variable until
 * awaken by any thread. Encryption can also be done in this function
 * @return Returns dSocket state for proper termination in case of connection failure
 */
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

        if (mTerminate.load() || !getSmartphoneWriteBuffer(Packet)) {
            break;
        }

        if (mConnected.load()) {
            while (WrittenTotal < mPacketSize) {
                Result = mSocketUDP -> writeUDP(Packet + WrittenTotal, mPacketSize - WrittenTotal, &WrittenBytes,
                                                reinterpret_cast <const sockaddr*>(&mSmartphoneAddr),
                                                mSmartphoneAddrLen);

                switch (Result) {
                    case dSocketResult::SUCCESS:
                        WrittenTotal += WrittenBytes;
                        break;
                    default:
                        ///---TODO: Add proper termination handling---///
                        //---There could be an error related to disconnection---//
                        //---Need to fix dSocket in the future---//
                        terminate();
                        return dSocketResult::WRITE_ERROR;
                }
            }

            WrittenTotal = 0;
        }

        mSmartphoneWriteState = false;
    }

    return dSocketResult::SUCCESS;
}
/**
 * @description
 * Function is used for parsing data from a packet and performing actions according to a parsed tag. Sleeps until
 * awakened by any thread
 * @return Returns dSocket state for proper termination in case of connection failure
 */
dSocketResult Server::smartphoneProcessCallback() {
    uint8_t Packet[mPacketSize];
    std::unique_lock <std::mutex> Lock(mSmartphoneProcessMutex);
    SmartphoneHeader Tag;
    std::pair <int32_t, int32_t> Pair;

    while (!mTerminate.load()) {
        mSmartphoneProcessCV.wait(Lock, [this] {
            return mSmartphoneProcessState || mTerminate.load();
        });

        if (mTerminate.load() || !getSmartphoneReadBuffer(Packet)) {
            break;
        }

        //----------//

        memcpy(&Tag, Packet, 4);

        if (Tag != SmartphoneHeader::REQUEST_CONN && !mConnected.load()) {
            std::scoped_lock <std::mutex> SmartphoneLock(mSmartphoneMutex);
            mSmartphoneAddr     = {};
            mSmartphoneAddrLen  = 0;

            mSmartphoneProcessState = false;

            continue;
        } else {
            switch (Tag) {
                case SmartphoneHeader::REQUEST_CONN:
                    mSmartphoneLastPingTime = std::chrono::system_clock::now();
                    mConnected.store(true);
                    std::cout << "Smartphone request" << std::endl;
                    break;
                case SmartphoneHeader::JOYSTICK_COORDS:
                    float PosX;
                    float PosY;

                    memcpy(&PosX, Packet + 4, 4);
                    memcpy(&PosY, Packet + 8, 4);

                    Pair = mMotorPWM -> getMotorsPWM(PosX, -PosY);
                    std::cout << Pair.first << " : " << Pair.second << std::endl;
                    mMonitorSTM -> sendPWM(Pair.first, Pair.second);

                    break;
                case SmartphoneHeader::PING:
                    mSmartphoneLastPingTime = std::chrono::system_clock::now();
                    std::cout << "Smartphone ping" << std::endl;

                    break;
                case SmartphoneHeader::STATUS:
                    ///---TODO: Add status handling---///

                    break;
                case SmartphoneHeader::INVALID:
                    std::cerr << "Invalid packet!" << std::endl;
                    break;
                case SmartphoneHeader::LATENCY:
                    std::cout << "Latency test from smartphone" << std::endl;
                    mMonitorSTM -> sendLatencyTest();

                    break;
                case SmartphoneHeader::DISCONNECTED:
                    break;
            }

            mSmartphoneProcessState = false;
        }
    }

    return dSocketResult::SUCCESS;
}
//-----------------------------//
/**
 * @description
 * Function is used for passing data obtained from socket in the read thread to the process thread. Should not be
 * used anywhere other than in <b>smartphoneReadCallback()</b> function
 * @param tBuffer Internal buffer from the <b>smartphoneReadCallback()</b> function
 * @return Returns <b>false</b>, if the program is being terminated, and <b>true</b> otherwise
 */
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
        std::scoped_lock <std::mutex> StateLock(mSmartphoneProcessMutex);       //---Lost wake up prevention---//
        mSmartphoneProcessState = true;
    }

    mSmartphoneReadBufferReady = true;

    mSmartphoneProcessCV.notify_one();
    mSmartphoneReadBufferCV.notify_one();

    return true;
}
/**
 * @description
 * Function is used for obtaining data from the read thread inside the process thread. Should not be
 * used anywhere other than in <b>smartphoneProcessCallback()</b> function
 * @param tBuffer Internal buffer from the <b>smartphoneProcessCallback()</b> function
 * @return Returns <b>false</b>, if the program is being terminated, and <b>true</b> otherwise
 */
bool Server::getSmartphoneReadBuffer(uint8_t* tBuffer) {
    std::unique_lock <std::mutex> Lock(mSmartphoneReadBufferMutex);

    mSmartphoneReadBufferCV.wait(Lock, [this] {
        return mSmartphoneReadBufferReady || mTerminate.load();
    });

    if (mTerminate.load()) {
        return false;
    }

    memcpy(tBuffer, mSmartphoneReadBuffer, mPacketSize);

    mSmartphoneReadBufferReady = false;

    mSmartphoneReadCV.notify_one();
    mSmartphoneReadBufferCV.notify_one();

    return true;
}

/**
 * @description
 * Function is used for passing data for writing to the socket. Can be called everywhere asynchronously, as long as
 * it safely puts all the data to the write buffer
 * @param tBuffer User-specified buffer to copy data from
 * @return Returns <b>false</b>, if the program is being terminated, and <b>true</b> otherwise
 */
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

    mSmartphoneWriteBufferReady = true;

    mSmartphoneWriteCV.notify_one();
    mSmartphoneWriteBufferCV.notify_one();

    return true;
}
/**
 * @description
 * Function is used for obtaining data for writing to the socket inside the write thread. Should not be
 * used anywhere other than in <b>smartphoneWriteCallback()</b> function
 * @param tBuffer Internal buffer from the <b>smartphoneWriteCallback()</b> function
 * @return Returns <b>false</b>, if the program is being terminated, and <b>true</b> otherwise
 */
bool Server::getSmartphoneWriteBuffer(uint8_t* tBuffer) {
    std::unique_lock <std::mutex> Lock(mSmartphoneWriteBufferMutex);

    mSmartphoneWriteBufferCV.wait(Lock, [this] {
        return mSmartphoneWriteBufferReady || mTerminate.load();
    });

    if (mTerminate.load()) {
        return false;
    }

    memcpy(tBuffer, mSmartphoneWriteBuffer, mPacketSize);

    mSmartphoneWriteBufferReady = false;

    mSmartphoneProcessCV.notify_one();
    mSmartphoneWriteBufferCV.notify_one();

    return true;
}
//-----------------------------//
/**
 * @description
 * Timer function is executed in a separate thread during server's uptime. The thread wakes up periodically to
 * checks if it is necessary to send ping to the serial port.
 */
ServerResult Server::timerCallback() {
    uint8_t Packet[mPacketSize];
    SmartphoneHeader Tag = SmartphoneHeader::PING;
    std::chrono::duration <double> Dur {};
    FILE* Stream;
    char StreamData[128];

    memcpy(Packet, &Tag, 4);

    while (!mTerminate.load()) {
        if (mConnected.load()) {
            Dur = std::chrono::system_clock::now() - mSmartphoneLastPingTime;

            if (Dur.count() * 1000 > mTimeoutMs) {
                mConnected.store(false);

                mSmartphoneAddr     = {};
                mSmartphoneAddrLen  = 0;

                if (mMonitorSTM != nullptr && mMonitorSTM -> isConnected()) {
                    mMonitorSTM -> sendStop();
                }
            }

            uint8_t ActiveSTM = mSerialActive.load();
            uint8_t ActiveRecording = mRecording.load();

            memcpy(Packet + 4, &ActiveSTM, 1);
            memcpy(Packet + 5, &ActiveRecording, 1);

            if (!fillSmartphoneWriteBuffer(Packet)) {
                break;
            }
        }

        //----------//

        ///---TODO: add bash command and status to the YAML config---///
        Dur = std::chrono::system_clock::now() - mRecordLastCheckTime;

        if (Dur.count() * 1000 > mRecordCheckTimeoutMs) {
            Stream = popen(mRecordStatusCmd.c_str(), "r");

            if (Stream != nullptr && fgets(StreamData, 128, Stream) != nullptr) {
                if (std::strstr(StreamData, "running")) {
                    mRecording.store(true);
                } else {
                    mRecording.store(false);
                }
            } else {
                mRecording.store(false);
            }

            if (pclose(Stream) == -1) {
                std::cerr << "timerCallback: failed to close the stream!" << std::endl;
            }

            mRecordLastCheckTime = std::chrono::system_clock::now();
        }

        //----------//

        std::this_thread::sleep_for(std::chrono::milliseconds(mTimerSleepIntervalMs));
    }

    return ServerResult::SUCCESS;
}
/**
 * @description
 * Function calls serial main loop therefore should be called in a separate thread
 */
void Server::serialCallback() {
    while (!mTerminate.load()) {
        mMessengerSTM               = new SerialMessenger;
        mMessengerSTM -> mBuffer    = new uint8_t[mSerialPacketSize];
        mMonitorSTM                 = new SerialMonitor(mSerialPath,
                                                        mSerialPacketSize,
                                                        mMessengerSTM,
                                                        mSerialTimeout,
                                                        mSerialTimerSleepIntervalMs,
                                                        mSerialPingIntervalMs);

        std::future <void> SerialDataThread;

        try {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            {
                std::scoped_lock <std::mutex> Lock(mMessengerSTM -> mMutex);
                mSerialActive.store(true);
            }

            SerialDataThread = std::async(std::launch::async, &Server::serialDataCallback, this);
            mMonitorSTM -> startSerialLoop();

            //----------//

            {
                std::scoped_lock <std::mutex> Lock(mMessengerSTM -> mMutex);
                mSerialActive.store(false);
            }

            mMessengerSTM -> mDataCV.notify_one();

            if (SerialDataThread.valid()) {
                SerialDataThread.wait();
            }

            delete[](mMessengerSTM -> mBuffer);
            delete(mMessengerSTM);
            delete(mMonitorSTM);
        } catch (const std::runtime_error& tExcept) {
            std::cerr << tExcept.what() << std::endl;

            {
                std::scoped_lock <std::mutex> Lock(mMessengerSTM -> mMutex);
                mSerialActive.store(false);
            }

            mMessengerSTM -> mDataCV.notify_one();

            if (SerialDataThread.valid()) {
                SerialDataThread.wait();
            }

            delete(mMessengerSTM);
            delete(mMonitorSTM);
        }
    }
}

///---TODO: reading data without connection can create vulnerability - fix it---///
void Server::serialDataCallback() {
    if (mMessengerSTM == nullptr) {
        return;
    }

    std::unique_lock <std::mutex> Lock(mMessengerSTM -> mMutex);
    auto StmTag = SerialMonitor::PacketType::INVALID;
    auto SmartphoneTag = SmartphoneHeader::INVALID;
    uint8_t Packet[mPacketSize];

    while (!mTerminate.load() && mSerialActive.load()) {
        mMessengerSTM -> mDataCV.wait(Lock, [this] {
            return mMessengerSTM -> mNewData.load() || mTerminate.load() || !mSerialActive.load();
        });

        if (!mSerialActive.load() || mTerminate.load()) {
            break;
        }

        memcpy(&StmTag, mMessengerSTM -> mBuffer, 4);

        if (StmTag == SerialMonitor::PacketType::LATENCY) {
            SmartphoneTag = SmartphoneHeader::LATENCY;
            memcpy(Packet, &SmartphoneTag, 4);
            fillSmartphoneWriteBuffer(Packet);
            std::cout << "Latency test from STM32" << std::endl;
        } else if (StmTag == SerialMonitor::PacketType::ENCODER) {
            SmartphoneTag = SmartphoneHeader::ENCODER;
            memcpy(Packet, &SmartphoneTag, 4);
            memcpy(Packet + 4, mMessengerSTM -> mBuffer + 4, 16);   //---Two double values for the left and the right encoders---//

            double Left;
            double Right;

            memcpy(&Left, mMessengerSTM -> mBuffer + 4, 8);
            memcpy(&Right, mMessengerSTM -> mBuffer + 12, 8);

            fillSmartphoneWriteBuffer(Packet);
            std::cout << "Encoder data from STM32: " << Left << ", " << Right << std::endl;
        }

        mMessengerSTM -> mNewData.store(false);
        StmTag = SerialMonitor::PacketType::INVALID;
    }
}