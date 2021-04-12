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
        return switch (tValue) {
            case 0 -> REQUEST_CONN;
            case 1 -> JOYSTICK_COORDS;
            case 2 -> PING;
            case 3 -> STATUS;
            case 4 -> INVALID;
            default -> null;
        };

    }

    public int getValue() {
        return mValue;
    }
}