//
// Created by devilox on 28.05.2021.
//
//-----------------------------//
#include "Akula.h"
//-----------------------------//
RingBuffer gReceiveBuffer = {
        .mHead = 0,
        .mTail = 0
};
//-----------------------------//
uint8_t ReadBuffer[PACKET_SIZE];
uint8_t WriteBuffer[PACKET_SIZE];

int gConnected                  = 0;
int gRunning                    = 1;

uint32_t TimeoutMs              = 5000;

uint32_t gSignalDiffLeft        = 0;
uint32_t gSignalDiffRight       = 0;

bool gFirstCapturedLeft         = false;
bool gFirstCapturedRight        = false;

uint32_t gFirstValLeft          = 0;
uint32_t gFirstValRight         = 0;

uint32_t gSecondValLeft         = 0;
uint32_t gSecondValRight        = 0;

double gFrequencyLeft           = 0.0;
double gFrequencyRight          = 0.0;

uint32_t gSendEncoderCounter    = 0;
//-----------------------------//
/**
 * @description
 * Function is used for continuous iteration trough a ring buffer
 * @param tCurrentIndex Index to increment
 * @return <b>tCurrentIndex + 1</b> if tCurrentIndex is less than <b>RING_BUFFER_SIZE</b>, <b>0</b> otherwise
 */
uint32_t ringNextIndex(uint32_t tCurrentIndex) {
    return ((tCurrentIndex) + 1) & (RING_BUFFER_SIZE - 1);
}
/**
 * @description
 * As long as reading in HAL is implemented trough a callback function, to get actual buffer without using mutexes
 * ring buffer is needed with a function like this to extract data
 * @param tBuffer Buffer to fill with bytes
 * @param tLength Desired number of bytes
 * @return Returns desired number of bytes if there are enough bytes in the ring buffer, less than that in case of
 * underrun, 0 otherwise
 */
uint32_t readSerial(uint8_t* tBuffer, uint32_t tLength) {
    uint32_t ReceivedBytes = 0;

    if ((tBuffer == NULL) || (tLength == 0)) {
        return 0;
    }

    while (tLength--) {
        if (gReceiveBuffer.mHead == gReceiveBuffer.mTail) {
            return ReceivedBytes;
        }

        ReceivedBytes++;

        *tBuffer++ = gReceiveBuffer.mBuffer[gReceiveBuffer.mTail];
        gReceiveBuffer.mTail = ringNextIndex(gReceiveBuffer.mTail);
    }

    return ReceivedBytes;
}
void clearBuffer() {
    gReceiveBuffer.mTail = gReceiveBuffer.mHead;
}