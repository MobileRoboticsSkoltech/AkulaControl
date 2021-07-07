//
// Created by devilox on 28.05.2021.
//
//-----------------------------//
#ifndef STM32_AKULA_H
#define STM32_AKULA_H
//-----------------------------//
#include <stdint.h>
#include <stddef.h>
//-----------------------------//
#define RING_BUFFER_SIZE    128
#define PACKET_SIZE			32
#define MAX_PWM             139
#define OFFSET_PWM          39
//-----------------------------//
typedef struct RingBuffer {
    uint32_t 	mHead;
    uint32_t 	mTail;

    uint8_t 	mBuffer[RING_BUFFER_SIZE];
} RingBuffer;

/**
 * @description
 * Values for <b>PacketType</b> start not from zero to prevents accidents with garbage data in the read buffer
 */
enum PacketType {
    REQUEST_CONN        = 0x0000AAAA,   /**< Used for connection establishing procedure */
    JOYSTICK_COORDS     = 0x0000AAAB,
    PING                = 0x0000AAAC,
    ENCODER             = 0x0000AAAD,
    LATENCY             = 0x0000AAAE,
    STOP                = 0x0000AAAF,   /**< Sets PWM to zero */
    INVALID             = 0x0000FFFE,   /**< Mostly used as a default value */
    SHUTDOWN            = 0X0000FFFF    /**< Added to match the same tag in the stm32 program, so sanitizer wouldn't highlight a warning about endless loop */
};
//-----------------------------//
extern RingBuffer gReceiveBuffer;
//-----------------------------//
extern uint8_t ReadBuffer[PACKET_SIZE];
extern uint8_t WriteBuffer[PACKET_SIZE];

extern int gConnected;
extern int gRunning;

extern uint32_t TimeoutMs;
//-----------------------------//
uint32_t ringNextIndex(uint32_t tCurrentIndex);
uint32_t readSerial(uint8_t* tBuffer, uint32_t tLength);
void clearBuffer();
//-----------------------------//
#endif
