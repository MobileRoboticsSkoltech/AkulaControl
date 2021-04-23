package com.example.androidapp;

public enum Header {
    REQUEST_CONN(0),
    JOYSTICK_COORDS(1),
    PING(2),
    STATUS(3),
    INVALID(4);

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
        } else {
            return null;
        }

    }

    public int getValue() {
        return mValue;
    }
}