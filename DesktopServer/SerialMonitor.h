//
// Created by devilox on 25.05.2021.
//
//-----------------------------//
#ifndef SERIALMONITOR_H
#define SERIALMONITOR_H
//-----------------------------//
#include <future>
//-----------------------------//
#include "SerialConnector.h"
//-----------------------------//
struct SerialMessenger {
    std::condition_variable     mDataCV;
    std::atomic_bool            mNewData        = false;
    std::mutex                  mMutex;
    uint8_t*                    mBuffer         = nullptr;
};
//-----------------------------//
/**
 * @description
 * The class implements methods and functionality for communication with a serial device for a
 * specific task which involves custom packets, timers, connection establishing procedure and (kinda) watchdog
 */
class SerialMonitor {
public:
    /**
     * @description
     * Values for <b>PacketType</b> start not from zero to prevents accidents with garbage data in the read buffer
     */
    enum class PacketType {
        REQUEST_CONN        = 0x0000AAAA,   /**< Used for connection establishing procedure */
        JOYSTICK_COORDS     = 0x0000AAAB,
        PING                = 0x0000AAAC,
        ENCODER             = 0x0000AAAD,
        LATENCY             = 0x0000AAAE,   /**< Used in an empty packet to measure the latency between smartphone and stm32 board */
        STOP                = 0x0000AAAF,   /**< Sets PWM to zero */
        INVALID             = 0x0000FFFE,   /**< Mostly used as a default value */
        SHUTDOWN            = 0X0000FFFF    /**< Added to match the same tag in the stm32 program, so sanitizer wouldn't highlight a warning about endless loop */
    };

    //----------//

    SerialMonitor(      const std::string& tSerialPath,
                        size_t tPacketSize,
                        SerialMessenger* tMessenger,
                        uint32_t tSerialTimeoutMs,
                        uint32_t tTimerSleepIntervalMs,
                        uint32_t tPingIntervalMs);
    ~SerialMonitor();

    //----------//

    void startSerialLoop();
    void terminate();

    //----------//

    void sendPWM(int32_t tLeftPWM, int32_t tRightPWM);
    void sendLatencyTest();
    void sendStop();

    //----------//

    bool isConnected();
private:
    SerialConnector*                            mConnector                  = nullptr;
    SerialMessenger*                            mMessenger                  = nullptr;

    std::future <void>                          mTimerThread;
    uint32_t                                    mTimerSleepIntervalMs       = 0;
    uint32_t                                    mSerialTimeout              = 0;

    uint8_t*                                    mReadBuffer                 = nullptr;
    uint8_t*                                    mWriteBuffer                = nullptr;

    size_t                                      mPacketSize                 = 0;
    std::string                                 mSerialPath;

    std::atomic_bool                            mRunning                    = false;
    std::atomic_bool                            mConnected                  = false;

    std::chrono::system_clock::time_point       mLastPing;
    std::chrono::duration <double>              mPingDuration               = {};
    uint32_t                                    mPingIntervalMs             = 0;

    //----------//

    void timerCallback();
};
//-----------------------------//
#endif
