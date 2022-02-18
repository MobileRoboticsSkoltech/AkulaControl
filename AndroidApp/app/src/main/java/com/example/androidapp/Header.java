package com.example.androidapp;
//-----------------------------//
/**
 * Header enum is used as packets' tag
 */
public enum Header {
    REQUEST_CONN(0x0000AAAA),
    JOYSTICK_COORDS(0x0000AAAB),
    PING(0x0000AAAC),
    STATUS(0x0000AAAD),
    DISCONNECTED(0x0000AAAE),
    LATENCY(0x0000AAAF),
    ENCODER(0x0000AABA),
    LATENCY_RESPONSE(0x0000AABB),
    STM32_ONLINE(0x0000AABC),
    STM32_DISCONNECTED(0x0000AABD),
    RECORD_ACTIVE(0x0000AABE),
    RECORD_INACTIVE(0x0000AABF),
    SENSOR_ACTIVE(0x0000AACA),
    SENSOR_INACTIVE(0x0000AACB),
    TOGGLE_RECORD(0x0000AACC),
    TOGGLE_SENSOR(0x0000AACD),
    INVALID(0x0000FFFF);

    private final int mValue;

    //----------//

    Header(int tValue) {
        this.mValue = tValue;
    }

    //----------//

    public static Header fromInt(int tValue) {
        if (tValue == 0x0000AAAA) {
            return REQUEST_CONN;
        } else if (tValue == 0x0000AAAB) {
            return JOYSTICK_COORDS;
        } else if (tValue == 0x0000AAAC) {
            return PING;
        } else if (tValue == 0x0000AAAD) {
            return STATUS;
        } else if (tValue == 0x0000AAAE) {
            return DISCONNECTED;
        } else if (tValue == 0x0000AAAF) {
            return LATENCY;
        } else if (tValue == 0x0000AABA) {
            return ENCODER;
        } else if (tValue == 0x0000AABB) {
            return LATENCY_RESPONSE;
        } else if (tValue == 0x0000AABC) {
            return STM32_ONLINE;
        } else if (tValue == 0x0000AABD) {
            return STM32_DISCONNECTED;
        } else if (tValue == 0x0000AABE) {
            return RECORD_ACTIVE;
        } else if (tValue == 0x0000AABF) {
            return RECORD_INACTIVE;
        } else if (tValue == 0x0000AACA) {
            return SENSOR_ACTIVE;
        } else if (tValue == 0x0000AACB) {
            return SENSOR_INACTIVE;
        } else if (tValue == 0x0000AACC) {
            return TOGGLE_RECORD;
        } else if (tValue == 0x0000AACD) {
            return TOGGLE_SENSOR;
        } else if (tValue == 0x0000FFFF) {
            return INVALID;
        } else {
            return null;
        }
    }

    public int getValue() {
        return mValue;
    }
}