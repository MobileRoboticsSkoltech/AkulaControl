package com.example.androidapp;

public enum Header {
    REQUEST_CONN(0),
    JOYSTICK_COORDS(1),
    PING(2),
    STATUS(3),
    INVALID(4),
    DISCONNECTED(5),
    LATENCY(6),
    ENCODER(7),
    LATENCY_RESPONSE(8),
    STM32_ONLINE(9),
    STM32_DISCONNECTED(10),
    RECORD_ACTIVE(11),
    RECORD_INACTIVE(12);

    private final int mValue;

    Header(int tValue) {
        this.mValue = tValue;
    }

    public static Header fromInt(int tValue) {
        if (tValue == 0) {
            return REQUEST_CONN;
        } else if (tValue == 1) {
            return JOYSTICK_COORDS;
        } else if (tValue == 2) {
            return PING;
        } else if (tValue == 3) {
            return STATUS;
        } else if (tValue == 4) {
            return INVALID;
        } else if (tValue == 5) {
            return DISCONNECTED;
        } else if (tValue == 6) {
            return LATENCY;
        } else if (tValue == 7) {
            return ENCODER;
        } else if (tValue == 8) {
            return LATENCY_RESPONSE;
        } else if (tValue == 9) {
            return STM32_ONLINE;
        } else if (tValue == 10) {
            return STM32_DISCONNECTED;
        } else if (tValue == 11) {
            return RECORD_ACTIVE;
        } else if (tValue == 12) {
            return RECORD_INACTIVE;
        } else {
            return null;
        }
    }

    public int getValue() {
        return mValue;
    }
}