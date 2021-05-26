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
///---TODO: maybe it's better to clear read buffer before processing packet---///
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
        INVALID             = 0x0000FFFE,   /**< Mostly used as a default value */
        SHUTDOWN            = 0X0000FFFF    /**< Added to match the same tag in the stm32 program, so sanitizer wouldn't highlight a warning about endless loop */
    };

    //----------//

    SerialMonitor(const std::string& tSerialPath, size_t tPacketSize);
    ~SerialMonitor();

    //----------//

    void startSerialLoop();

    //----------//

    void sendCoords();
private:
    SerialConnector*                            mConnector                  = nullptr;

    ///---TODO: change future return value---///
    std::future <void>                          mTimerThread;

    size_t                                      mPacketSize                 = 0;

    uint8_t*                                    mReadBuffer                 = nullptr;
    uint8_t*                                    mWriteBuffer                = nullptr;

    std::atomic_bool                            mRunning                    = false;
    std::atomic_bool                            mConnected                  = false;

    std::chrono::system_clock::time_point       mLastPing;
    std::chrono::duration <double>              mPingDuration               = {};
    uint32_t                                    mPingIntervalMs             = 1000;

    //----------//

    void timerCallback();
};
//-----------------------------//
#endif
